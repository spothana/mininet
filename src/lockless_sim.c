/* lockless_sim.c
 * Multi-core lockless simulation — Part 3.
 *
 * Hot-path synchronisation summary (zero mutexes):
 *
 *   Generator → AP worker   : MPSC queue (atomic_exchange on tail)
 *   AP worker  → Relay       : SPSC ring  (atomic store/load on tail/head)
 *   Relay      → Client RX  : SPSC ring  (atomic store/load per client)
 *   RCU table  lookup        : single atomic_load_acquire (no lock)
 *   Per-core stats           : AlignedCounter (relaxed atomic, no sharing)
 *
 * Shutdown uses atomic_store(stop, 1) + sched_yield() spin exit —
 * no condvar broadcast needed because threads poll their SPSC/MPSC.
 */
#include "mininet.h"

/* ---- forward declarations for affinity helper ---- */
extern int pin_thread_to_core(int core_id);
extern int num_online_cores(void);
extern void aligned_counter_add(AlignedCounter *c, uint_fast64_t n);
extern uint_fast64_t aligned_counter_load(const AlignedCounter *c);
extern void aligned_counter_init(AlignedCounter *c);

/* ============================================================
 * LOCKLESS AP WORKER THREAD
 *
 * Loop:
 *   1. Drain MPSC queue (packets from generator).
 *      For each: find which client owns src_mac, push into
 *      that client's SPSC tx_ring (relay_ring direction).
 *      (In this model, generator → MPSC → worker → relay_ring)
 *   2. Drain each client's SPSC tx_ring (round-robin).
 *      For each: run scheduler key, forward to relay_ring.
 *   3. sched_yield() if nothing to do — no condvar, no sleep.
 *      Pinned threads yield instead of sleeping so the OS
 *      can give the core to another thread if needed, but
 *      return quickly when work arrives.
 * ============================================================ */

static void *lockless_ap_thread(void *arg) {
    LocklessAP *w = (LocklessAP *)arg;

    /* Pin to designated core */
    if (pin_thread_to_core(w->cpu_id) == 0)
        slog_append(w->slog, "[LAP%d] pinned to core %d", w->ap_id, w->cpu_id);
    else
        slog_append(w->slog, "[LAP%d] running (no pin)", w->ap_id);

    while (!atomic_load_explicit(&w->stop, memory_order_acquire)) {

        int did_work = 0;

        /* ── Drain MPSC (generator → this AP) ── */
        Packet pkt;
        int attempts = 0;
        while (attempts < 8) {   /* bounded drain per iteration */
            if (mpsc_pop(&w->mpsc_in, &pkt) != 0) break;
            did_work = 1;

            /* Forward straight to relay_ring (SPSC, we are sole producer) */
            if (spsc_push(&w->relay_ring, &pkt) != 0) {
                aligned_counter_add(&w->packets_dropped, 1);
            } else {
                aligned_counter_add(&w->packets_tx, 1);
            }
            attempts++;
        }

        /* ── Drain per-client TX SPSCs (client devices → AP) ── */
        for (int i = 0; i < w->client_count; i++) {
            SpscRing *ring = w->tx_rings[i];
            if (!ring) continue;
            if (spsc_pop(ring, &pkt) == 0) {
                did_work = 1;
                pkt.ap_id = w->ap_id;
                if (spsc_push(&w->relay_ring, &pkt) != 0)
                    aligned_counter_add(&w->packets_dropped, 1);
                else
                    aligned_counter_add(&w->packets_tx, 1);
            }
        }

        if (!did_work) sched_yield();
    }

    slog_append(w->slog, "[LAP%d] stopped. tx=%llu dropped=%llu",
                w->ap_id,
                (unsigned long long)aligned_counter_load(&w->packets_tx),
                (unsigned long long)aligned_counter_load(&w->packets_dropped));
    return NULL;
}

void lockless_ap_init(LocklessAP *w, int ap_id, int cpu_id,
                      AccessPoint *ap, RcuTable *rcu_ct, SafeLog *slog) {
    memset(w, 0, sizeof(LocklessAP));
    w->ap_id  = ap_id;
    w->cpu_id = cpu_id;
    w->ap     = ap;
    w->rcu_ct = rcu_ct;
    w->slog   = slog;
    atomic_store_explicit(&w->stop, 0, memory_order_relaxed);
    aligned_counter_init(&w->packets_tx);
    aligned_counter_init(&w->packets_rx);
    aligned_counter_init(&w->packets_dropped);
    mpsc_init(&w->mpsc_in);
    spsc_init(&w->relay_ring);
    sched_init(&w->scheduler);
}

void lockless_ap_add_client(LocklessAP *w, const uint8_t mac[6],
                             const char *name, SpscRing *tx, SpscRing *rx) {
    int i = w->client_count;
    if (i >= MAX_LOCKLESS_CLIENTS) return;
    memcpy(w->client_macs[i], mac, 6);
    strncpy(w->client_names[i], name, 31);
    w->tx_rings[i] = tx;
    w->rx_rings[i] = rx;
    w->client_count++;
}

void lockless_ap_start(LocklessAP *w) {
    pthread_create(&w->tid, NULL, lockless_ap_thread, w);
}

void lockless_ap_stop(LocklessAP *w) {
    atomic_store_explicit(&w->stop, 1, memory_order_release);
    pthread_join(w->tid, NULL);
    mpsc_drain(&w->mpsc_in);
}

/* ============================================================
 * LOCKLESS GENERATOR THREAD
 *
 * Pushes via MPSC queue — no mutex, no condvar.
 * Uses atomic_fetch_add for seq counter (lock-free).
 * Targets APs round-robin based on client's ap_id.
 * ============================================================ */

static const char *ll_payloads[] = {
    "CarPlay-I-frame", "CarPlay-P-frame",
    "AndroidAuto-AAC", "AndroidAuto-touch",
    "OTA-chunk",       "DHCP-renew",
    "DNS-query",       "TCP-ACK",
    "HTTPS-data",      "mDNS-announce",
};
#define LL_N_PAYLOADS 10

static uint8_t ll_bcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static void *lockless_gen_thread(void *arg) {
    LocklessGen *g = (LocklessGen *)arg;
    slog_append(g->slog, "[LGEN] generator thread started");

    while (!atomic_load_explicit(&g->stop, memory_order_acquire)) {
        uint32_t seq     = atomic_fetch_add_explicit(&g->seq_counter, 1,
                                                     memory_order_relaxed);
        int      ap_idx  = (int)(seq % (uint32_t)g->worker_count);
        LocklessAP *w    = &g->workers[ap_idx];

        const char *payload = ll_payloads[seq % LL_N_PAYLOADS];

        Packet pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.seq = seq;
        /* src_mac: use AP's first client mac for identification */
        if (w->client_count > 0)
            memcpy(pkt.src_mac, w->client_macs[seq % (uint32_t)w->client_count], 6);
        memcpy(pkt.dst_mac, ll_bcast, 6);
        pkt.len = (uint16_t)strlen(payload);
        memcpy(pkt.data, payload, pkt.len);
        pkt.timestamp_ms = now_ms();
        pkt.ap_id = ap_idx;

        /*
         * MPSC push — atomic_exchange on tail, no mutex.
         * If the relay ring is full, this packet will be
         * dropped in the AP worker (non-blocking path).
         */
        mpsc_push(&w->mpsc_in, &pkt);

        sleep_us(GEN_TICK_US / 2);  /* generate at 2× rate for demo */
    }

    slog_append(g->slog, "[LGEN] stopped. seq_hi=%u",
                atomic_load_explicit(&g->seq_counter, memory_order_relaxed));
    return NULL;
}

void lockless_gen_start(LocklessGen *g) {
    atomic_store_explicit(&g->stop, 0, memory_order_relaxed);
    atomic_store_explicit(&g->seq_counter, 5000, memory_order_relaxed);
    pthread_create(&g->tid, NULL, lockless_gen_thread, g);
}

void lockless_gen_stop(LocklessGen *g) {
    atomic_store_explicit(&g->stop, 1, memory_order_release);
    pthread_join(g->tid, NULL);
}

/* ============================================================
 * LOCKLESS RELAY THREAD
 *
 * Drains each AP's relay_ring (SPSC — relay is sole consumer).
 * RCU lookup for dst_mac → client RX ring assignment.
 * Pushes downlink frame into per-client rx_ring (SPSC —
 * relay is sole producer per ring).
 * No mutex anywhere on the hot path.
 * ============================================================ */

static void *lockless_relay_thread(void *arg) {
    LocklessRelay *r = (LocklessRelay *)arg;
    slog_append(r->slog, "[LRELAY] relay thread started");

    while (!atomic_load_explicit(&r->stop, memory_order_acquire)) {
        int did_work = 0;

        for (int i = 0; i < r->worker_count; i++) {
            LocklessAP *w = &r->workers[i];
            Packet pkt;

            /*
             * spsc_pop: relay is the SOLE consumer of relay_ring.
             * No lock — just load head (relaxed) and tail (acquire).
             */
            while (spsc_pop(&w->relay_ring, &pkt) == 0) {
                did_work = 1;
                aligned_counter_add(&r->packets_relayed, 1);

                /* BFS: find least-interfering neighbour */
                const AccessPoint *ap = &r->graph->aps[pkt.ap_id];
                int best_nb  = -1, best_pct = 101;
                for (int e = 0; e < ap->neighbor_count; e++) {
                    if (ap->neighbors[e].interference_pct < best_pct) {
                        best_pct = ap->neighbors[e].interference_pct;
                        best_nb  = ap->neighbors[e].neighbor_id;
                    }
                }

                char src[18]; mac_to_str(pkt.src_mac, src);
                slog_append(r->slog,
                    "[LTX] seq=%-5u AP%d->AP%d (%d%%) src=%s '%s'",
                    pkt.seq, pkt.ap_id,
                    best_nb >= 0 ? best_nb : pkt.ap_id,
                    best_nb >= 0 ? best_pct : 0,
                    src, pkt.data);

                /*
                 * RCU lookup — zero locks.
                 * atomic_load_acquire on the bucket array pointer,
                 * then plain chain walk on immutable data.
                 */
                RcuEntry *entry = rcu_table_lookup(r->rcu_ct, pkt.src_mac);

                /* Build downlink frame */
                int target_ap = (best_nb >= 0) ? best_nb : pkt.ap_id;
                Packet rx_pkt = pkt;
                memcpy(rx_pkt.dst_mac, pkt.src_mac, 6);
                memcpy(rx_pkt.src_mac, ll_bcast,    6);
                rx_pkt.ap_id = target_ap;
                rx_pkt.seq  |= 0x80000000u;

                /*
                 * Push into the client's rx_ring via SPSC.
                 * Relay is the sole producer — no contention.
                 * Find which client slot on the target AP owns this MAC.
                 */
                LocklessAP *tap = &r->workers[target_ap];
                int pushed = 0;
                for (int ci = 0; ci < tap->client_count; ci++) {
                    if (memcmp(tap->client_macs[ci], rx_pkt.dst_mac, 6)==0) {
                        if (tap->rx_rings[ci] &&
                            spsc_push(tap->rx_rings[ci], &rx_pkt) == 0) {
                            aligned_counter_add(&tap->packets_rx, 1);
                            if (entry) {
                                slog_append(r->slog,
                                    "[LRX] %-20s rx_ring <- seq=%u '%s'",
                                    entry->name,
                                    rx_pkt.seq & 0x7FFFFFFFu,
                                    rx_pkt.data);
                            }
                            pushed = 1;
                        }
                        break;
                    }
                }
                (void)pushed;
            }
        }

        if (!did_work) sched_yield();
    }

    slog_append(r->slog, "[LRELAY] stopped. relayed=%llu",
                (unsigned long long)aligned_counter_load(&r->packets_relayed));
    return NULL;
}

void lockless_relay_start(LocklessRelay *r) {
    atomic_store_explicit(&r->stop, 0, memory_order_relaxed);
    aligned_counter_init(&r->packets_relayed);
    pthread_create(&r->tid, NULL, lockless_relay_thread, r);
}

void lockless_relay_stop(LocklessRelay *r) {
    atomic_store_explicit(&r->stop, 1, memory_order_release);
    pthread_join(r->tid, NULL);
}

/* ============================================================
 * sim_run_lockless — Part 3 entry point
 * ============================================================ */

void sim_run_lockless(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  LOCKLESS MULTI-CORE SIMULATION  (Part 3)            ║\n");
    printf("║  Zero mutexes on hot path                            ║\n");
    printf("║  SPSC ring · MPSC queue · RCU table · CPU affinity   ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");

    int ncores = num_online_cores();
    printf("\n[Setup] %d CPU cores available\n", ncores);

    /* ── shared infrastructure ── */
    SafeLog  slog;
    APGraph  graph;
    RcuTable rcu_ct;

    slog_init(&slog);
    graph_init(&graph);
    rcu_table_init(&rcu_ct);

    int id0 = graph_add_ap(&graph, "LL-Main",  BAND_5G, 36, 20);
    int id1 = graph_add_ap(&graph, "LL-North", BAND_5G, 40, 18);
    int id2 = graph_add_ap(&graph, "LL-South", BAND_2G,  6, 15);
    graph_add_edge(&graph, id0, id1, 30);
    graph_add_edge(&graph, id0, id2, 10);
    graph_add_edge(&graph, id1, id2, 20);

    printf("[Setup] AP graph: 3 APs\n");

    /* ── per-client SPSC rings (allocated on heap, 64-byte aligned) ── */
#define N_CLIENTS 6
    SpscRing *tx_rings[N_CLIENTS];
    SpscRing *rx_rings[N_CLIENTS];
    for (int i = 0; i < N_CLIENTS; i++) {
        tx_rings[i] = aligned_alloc(CACHE_LINE_SIZE, sizeof(SpscRing));
        rx_rings[i] = aligned_alloc(CACHE_LINE_SIZE, sizeof(SpscRing));
        spsc_init(tx_rings[i]);
        spsc_init(rx_rings[i]);
    }

    /* ── client MACs and metadata ── */
    uint8_t macs[N_CLIENTS][6] = {
        {0xAA,0xBB,0xCC,0x11,0x22,0x33},
        {0xAA,0xBB,0xCC,0x44,0x55,0x66},
        {0xDE,0xAD,0xBE,0xEF,0x00,0x01},
        {0xDE,0xAD,0xBE,0xEF,0x00,0x02},
        {0xCA,0xFE,0xBA,0xBE,0x00,0x01},
        {0x10,0x20,0x30,0x40,0x50,0x60},
    };
    const char *names[N_CLIENTS] = {
        "iPhone-CarPlay","Pixel-AndroidAuto",
        "iPad-Tablet","Samsung-Phone",
        "MacBook-Laptop","ESP32-IoT"
    };
    const char *ips[N_CLIENTS] = {
        "192.168.2.10","192.168.2.11",
        "192.168.2.20","192.168.2.21",
        "192.168.2.30","192.168.2.40"
    };
    int signals[N_CLIENTS] = {-42,-48,-61,-72,-55,-80};
    /* ap assignment: 0→AP0, 1→AP0, 2→AP1, 3→AP2, 4→AP0, 5→AP2 */
    int ap_ids[N_CLIENTS] = {id0,id0,id1,id2,id0,id2};

    /* ── populate RCU table (writer path, done before threads start) ── */
    printf("[Setup] Inserting %d clients into RCU table...\n", N_CLIENTS);
    for (int i = 0; i < N_CLIENTS; i++) {
        rcu_table_insert(&rcu_ct, macs[i], names[i], ips[i],
                         ap_ids[i], signals[i]);
    }
    printf("[Setup] RCU table ready (%d entries)\n", rcu_ct.count);

    /* ── init AP workers, one per AP ── */
    LocklessAP workers[3];
    for (int i = 0; i < 3; i++) {
        int core = i % ncores;  /* wrap if fewer cores than APs */
        lockless_ap_init(&workers[i], i, core, &graph.aps[i], &rcu_ct, &slog);
    }

    /* Assign clients to workers with their SPSC rings */
    for (int i = 0; i < N_CLIENTS; i++) {
        lockless_ap_add_client(&workers[ap_ids[i]], macs[i], names[i],
                               tx_rings[i], rx_rings[i]);
    }

    printf("[Setup] AP→core mapping:\n");
    for (int i = 0; i < 3; i++)
        printf("  AP%d '%s' → core %d  (%d clients)\n",
               i, workers[i].ap->ssid, workers[i].cpu_id,
               workers[i].client_count);

    /* ── relay ── */
    LocklessRelay relay = {
        .workers      = workers,
        .worker_count = 3,
        .graph        = &graph,
        .rcu_ct       = &rcu_ct,
        .slog         = &slog,
    };

    /* ── generator ── */
    LocklessGen gen = {
        .workers      = workers,
        .worker_count = 3,
        .slog         = &slog,
    };

    /* ── START ── */
    printf("\n[Lockless sim] Running for %d seconds...\n", SIM_DURATION_S);
    printf("[Hot path] Generator→MPSC→APWorker→SPSC→Relay→SPSC→RX\n");
    printf("[RCU]      Lookup: single acquire load, zero mutex\n\n");

    lockless_relay_start(&relay);
    for (int i = 0; i < 3; i++) lockless_ap_start(&workers[i]);
    lockless_gen_start(&gen);

    sleep(SIM_DURATION_S);

    /* ── STOP ── */
    printf("\n[Lockless sim] Stopping...\n");
    lockless_gen_stop(&gen);
    for (int i = 0; i < 3; i++) lockless_ap_stop(&workers[i]);
    lockless_relay_stop(&relay);

    /* ── print log + stats ── */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  Event Log                                           ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    slog_print(&slog);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  Per-core stats  (AlignedCounter — no false sharing) ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    printf("  Relay forwarded : %llu\n",
           (unsigned long long)aligned_counter_load(&relay.packets_relayed));
    printf("\n  %-4s %-10s %8s %8s %8s\n","AP","SSID","TX","RX","Dropped");
    printf("  %s\n","----------------------------------------------");
    for (int i = 0; i < 3; i++) {
        printf("  AP%d  %-10s %8llu %8llu %8llu\n", i,
               workers[i].ap->ssid,
               (unsigned long long)aligned_counter_load(&workers[i].packets_tx),
               (unsigned long long)aligned_counter_load(&workers[i].packets_rx),
               (unsigned long long)aligned_counter_load(&workers[i].packets_dropped));
    }

    /* ── cleanup ── */
    for (int i = 0; i < N_CLIENTS; i++) {
        free(tx_rings[i]);
        free(rx_rings[i]);
    }
    rcu_table_free(&rcu_ct);
    slog_free(&slog);

    printf("\nLockless simulation complete.\n\n");
}
