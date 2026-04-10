/* threaded_sim.c
 * Multi-threaded simulation: AP worker threads, packet generator thread,
 * relay/monitor thread — all wired together with mutexes and semaphores.
 *
 * Thread layout:
 *
 *   [GeneratorThread]
 *       Every GEN_TICK_US: pick random client, build packet,
 *       lock client->tx_mutex, enqueue to client->tx_queue, unlock.
 *       Also locks ap_worker->sched_mutex to insert client into heap
 *       if not already scheduled.
 *
 *   [APWorker Thread] × N  (one per Access Point)
 *       Every AP_TICK_US:
 *         lock sched_mutex
 *           pop best client from heap (signal+priority)
 *           lock client->tx_mutex
 *             dequeue packet from client->tx_queue
 *           unlock client->tx_mutex
 *           sq_try_produce() → outbound SharedQueue
 *           re-insert client into heap if queue non-empty
 *         unlock sched_mutex
 *
 *   [RelayThread]
 *       Every RELAY_TICK_US:
 *         for each AP worker: sq_try_consume() from outbound queue
 *           log the packet, increment relay counter
 *           use BFS on AP graph to find best neighbour
 *           (simulates 802.11r fast BSS transition decision)
 *
 *   Shutdown:
 *       Main sets atomic_int stop=1 on all contexts, then
 *       pthread_join() waits for clean exit of each thread.
 */
#include "mininet.h"

/* ================================================================
 * AP WORKER THREAD
 * ================================================================ */

static void *ap_worker_thread(void *arg) {
    APWorker *w = (APWorker *)arg;
    char mac_s[18];

    slog_append(w->slog, "[AP%d '%s'] thread started (tid=%lu)",
                w->ap_id, w->ap->ssid, (unsigned long)w->tid);

    while (!atomic_load(&w->stop)) {

        pthread_mutex_lock(&w->sched_mutex);

        /* --- scheduler tick --- */
        Client *winner = sched_pop_best(&w->scheduler);
        if (winner) {
            Packet pkt;
            int got_pkt = 0;

            /* Lock the client's own TX mutex to dequeue one packet */
            pthread_mutex_lock(&winner->tx_mutex);
            if (!pq_is_empty(&winner->tx_queue)) {
                pq_dequeue(&winner->tx_queue, &pkt);
                got_pkt = 1;
            }
            /* Re-insert into scheduler only if more packets remain */
            if (!pq_is_empty(&winner->tx_queue)) {
                sched_insert(&w->scheduler, winner);
            }
            pthread_mutex_unlock(&winner->tx_mutex);

            pthread_mutex_unlock(&w->sched_mutex);

            if (got_pkt) {
                pkt.ap_id = w->ap_id;
                /* Push to relay via shared queue (non-blocking) */
                if (sq_try_produce(w->outbound, &pkt) == 0) {
                    atomic_fetch_add(&w->packets_tx, 1);
                    winner->bytes_sent += pkt.len;
                } else {
                    atomic_fetch_add(&w->packets_dropped, 1);
                    winner->bytes_dropped += pkt.len;
                    mac_to_str(winner->mac, mac_s);
                    slog_append(w->slog,
                        "[AP%d] relay queue full — dropped seq=%u from %s",
                        w->ap_id, pkt.seq, mac_s);
                }
            }
        } else {
            pthread_mutex_unlock(&w->sched_mutex);
        }

        sleep_us(AP_TICK_US);
    }

    slog_append(w->slog, "[AP%d '%s'] thread stopping. tx=%llu dropped=%llu",
                w->ap_id, w->ap->ssid,
                (unsigned long long)atomic_load(&w->packets_tx),
                (unsigned long long)atomic_load(&w->packets_dropped));
    return NULL;
}

void ap_worker_init(APWorker *w, int ap_id, AccessPoint *ap,
                    SharedQueue *outbound, SafeLog *slog) {
    memset(w, 0, sizeof(APWorker));
    w->ap_id   = ap_id;
    w->ap      = ap;
    w->outbound = outbound;
    w->slog    = slog;
    atomic_store(&w->stop, 0);
    atomic_store(&w->packets_tx, 0);
    atomic_store(&w->packets_dropped, 0);
    pthread_mutex_init(&w->sched_mutex, NULL);
    sched_init(&w->scheduler);
}

void ap_worker_add_client(APWorker *w, Client *c) {
    if (w->client_count >= MAX_CLIENTS) return;
    w->clients[w->client_count++] = c;
    /* Pre-load into scheduler */
    pthread_mutex_lock(&w->sched_mutex);
    sched_insert(&w->scheduler, c);
    pthread_mutex_unlock(&w->sched_mutex);
}

void ap_worker_start(APWorker *w) {
    pthread_create(&w->tid, NULL, ap_worker_thread, w);
}

void ap_worker_stop(APWorker *w) {
    atomic_store(&w->stop, 1);
    pthread_join(w->tid, NULL);
    pthread_mutex_destroy(&w->sched_mutex);
}

/* ================================================================
 * PACKET GENERATOR THREAD
 * Simulates client devices producing traffic continuously.
 * Uses atomic seq_counter for lock-free sequence numbering.
 * ================================================================ */

static const char *sample_payloads[] = {
    "CarPlay-video-I-frame",
    "CarPlay-video-P-frame",
    "AndroidAuto-audio-AAC",
    "AndroidAuto-touch-event",
    "OTA-firmware-chunk",
    "DHCP-renew-request",
    "DNS-query",
    "TCP-ACK",
    "HTTPS-data",
    "mDNS-announce",
};
#define N_PAYLOADS 10

static uint8_t ap_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static void *generator_thread(void *arg) {
    GeneratorCtx *g = (GeneratorCtx *)arg;
    slog_append(g->slog, "[GEN] generator thread started");

    while (!atomic_load(&g->stop)) {
        /* Pick a random client */
        int idx = (int)(now_ms() % (uint64_t)g->client_count);
        Client *c = g->clients[idx];

        /* Build packet — no lock needed for reading static client fields */
        uint32_t seq = atomic_fetch_add(&g->seq_counter, 1);
        const char *payload = sample_payloads[seq % N_PAYLOADS];

        Packet pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.seq = seq;
        memcpy(pkt.src_mac, c->mac, 6);
        memcpy(pkt.dst_mac, ap_mac, 6);
        pkt.len = (uint16_t)strlen(payload);
        memcpy(pkt.data, payload, pkt.len);
        pkt.timestamp_ms = now_ms();
        pkt.ap_id = c->ap_id;

        /* Enqueue under client TX mutex */
        pthread_mutex_lock(&c->tx_mutex);
        pq_enqueue(&c->tx_queue, &pkt);
        pthread_mutex_unlock(&c->tx_mutex);

        /* Tell the owning AP worker to schedule this client */
        APWorker *w = &g->workers[c->ap_id];
        pthread_mutex_lock(&w->sched_mutex);
        /* Only insert if not already in heap (simple check: scan) */
        int found = 0;
        for (int i = 0; i < w->scheduler.size; i++) {
            if (w->scheduler.clients[i] == c) { found = 1; break; }
        }
        if (!found) sched_insert(&w->scheduler, c);
        pthread_mutex_unlock(&w->sched_mutex);

        sleep_us(GEN_TICK_US);
    }

    slog_append(g->slog, "[GEN] generator thread stopping. seq_hi=%u",
                atomic_load(&g->seq_counter));
    return NULL;
}

void generator_start(GeneratorCtx *g) {
    atomic_store(&g->stop, 0);
    atomic_store(&g->seq_counter, 1000);
    pthread_create(&g->tid, NULL, generator_thread, g);
}

void generator_stop(GeneratorCtx *g) {
    atomic_store(&g->stop, 1);
    pthread_join(g->tid, NULL);
}

/* ================================================================
 * RELAY / MONITOR THREAD
 * Drains all AP outbound queues. Uses BFS on the AP graph to
 * decide if a packet should be roamed to a neighbour AP.
 * ================================================================ */

static void *relay_thread(void *arg) {
    RelayCtx *r = (RelayCtx *)arg;
    slog_append(r->slog, "[RELAY] relay thread started");

    while (!atomic_load(&r->stop)) {
        for (int i = 0; i < r->worker_count; i++) {
            Packet pkt;
            /* Non-blocking drain — keep going while packets are available */
            while (sq_try_consume(&r->workers[i].outbound[0], &pkt) == 0) {
                atomic_fetch_add(&r->packets_relayed, 1);

                /* BFS from this AP: find best neighbour with lowest
                 * interference (simulate roaming / load-balance decision) */
                const AccessPoint *ap = &r->graph->aps[pkt.ap_id];
                int best_nb = -1;
                int best_pct = 101;
                for (int e = 0; e < ap->neighbor_count; e++) {
                    if (ap->neighbors[e].interference_pct < best_pct) {
                        best_pct = ap->neighbors[e].interference_pct;
                        best_nb  = ap->neighbors[e].neighbor_id;
                    }
                }

                char src[18]; mac_to_str(pkt.src_mac, src);
                if (best_nb >= 0) {
                    slog_append(r->slog,
                        "[RELAY] seq=%-5u AP%d->AP%d (interf=%d%%) src=%s payload='%s'",
                        pkt.seq, pkt.ap_id, best_nb, best_pct, src, pkt.data);
                } else {
                    slog_append(r->slog,
                        "[RELAY] seq=%-5u AP%d (no neighbours) src=%s",
                        pkt.seq, pkt.ap_id, src);
                }
            }
        }
        sleep_us(RELAY_TICK_US);
    }

    slog_append(r->slog, "[RELAY] relay thread stopping. relayed=%llu",
                (unsigned long long)atomic_load(&r->packets_relayed));
    return NULL;
}

void relay_start(RelayCtx *r) {
    atomic_store(&r->stop, 0);
    atomic_store(&r->packets_relayed, 0);
    pthread_create(&r->tid, NULL, relay_thread, r);
}

void relay_stop(RelayCtx *r) {
    atomic_store(&r->stop, 1);
    pthread_join(r->tid, NULL);
}

/* ================================================================
 * sim_run_threaded — wires everything together
 * ================================================================ */

void sim_run_threaded(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║         THREADED SIMULATION                          ║\n");
    printf("║  Threads: 1 generator + N AP workers + 1 relay       ║\n");
    printf("║  Sync:    mutex · semaphore · atomic                  ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");

    /* ---- 1. Global shared state ---- */
    SafeLog    slog;
    APGraph    graph;
    RouteTable route;
    SharedQueue outbound; /* single relay queue — all APs share it */
    ClientTable ct;

    slog_init(&slog);
    graph_init(&graph);
    rt_init(&route);
    sq_init(&outbound);
    ct_init(&ct);

    /* ---- 2. Build AP graph ---- */
    int id0 = graph_add_ap(&graph, "FordEVDD-Main",  BAND_5G, 36, 20);
    int id1 = graph_add_ap(&graph, "FordEVDD-North", BAND_5G, 40, 18);
    int id2 = graph_add_ap(&graph, "FordEVDD-South", BAND_2G,  6, 15);
    graph_add_edge(&graph, id0, id1, 30);
    graph_add_edge(&graph, id0, id2, 10);
    graph_add_edge(&graph, id1, id2, 20);

    printf("\n[Setup] AP graph:\n");
    graph_print(&graph);

    /* ---- 3. Init AP workers ---- */
    APWorker workers[3];
    ap_worker_init(&workers[0], id0, &graph.aps[id0], &outbound, &slog);
    ap_worker_init(&workers[1], id1, &graph.aps[id1], &outbound, &slog);
    ap_worker_init(&workers[2], id2, &graph.aps[id2], &outbound, &slog);

    /* ---- 4. Create clients ---- */
    uint8_t mac0[6]={0xAA,0xBB,0xCC,0x11,0x22,0x33};
    uint8_t mac1[6]={0xAA,0xBB,0xCC,0x44,0x55,0x66};
    uint8_t mac2[6]={0xDE,0xAD,0xBE,0xEF,0x00,0x01};
    uint8_t mac3[6]={0xDE,0xAD,0xBE,0xEF,0x00,0x02};
    uint8_t mac4[6]={0xCA,0xFE,0xBA,0xBE,0x00,0x01};
    uint8_t mac5[6]={0x10,0x20,0x30,0x40,0x50,0x60};

    Client *c0 = client_new(mac0,"192.168.1.10","iPhone-CarPlay",   -42,9,BAND_5G,36,id0);
    Client *c1 = client_new(mac1,"192.168.1.11","Pixel-AndroidAuto",-48,8,BAND_5G,36,id0);
    Client *c2 = client_new(mac2,"192.168.1.20","iPad-Tablet",      -61,5,BAND_5G,40,id1);
    Client *c3 = client_new(mac3,"192.168.1.21","Samsung-Phone",    -72,4,BAND_2G, 6,id2);
    Client *c4 = client_new(mac4,"192.168.1.30","MacBook-Laptop",   -55,6,BAND_5G,36,id0);
    Client *c5 = client_new(mac5,"192.168.1.40","ESP32-IoT",        -80,1,BAND_2G, 6,id2);

    Client *all[6] = {c0,c1,c2,c3,c4,c5};

    ct_insert(&ct, c0); ct_insert(&ct, c1); ct_insert(&ct, c2);
    ct_insert(&ct, c3); ct_insert(&ct, c4); ct_insert(&ct, c5);

    /* Assign clients to AP workers */
    ap_worker_add_client(&workers[id0], c0);
    ap_worker_add_client(&workers[id0], c1);
    ap_worker_add_client(&workers[id0], c4);
    ap_worker_add_client(&workers[id1], c2);
    ap_worker_add_client(&workers[id2], c3);
    ap_worker_add_client(&workers[id2], c5);

    printf("\n[Setup] Clients:\n");
    ct_print(&ct);

    /* ---- 5. Routing table ---- */
    rt_insert(&route, "192.168.1.10", mac0);
    rt_insert(&route, "192.168.1.11", mac1);
    rt_insert(&route, "192.168.1.20", mac2);
    rt_insert(&route, "192.168.1.21", mac3);
    rt_insert(&route, "192.168.1.30", mac4);
    rt_insert(&route, "192.168.1.40", mac5);

    printf("\n[Setup] Routing table:\n");
    rt_print(&route);

    /* ---- 6. Relay context ---- */
    RelayCtx relay = {
        .workers      = workers,
        .worker_count = 3,
        .graph        = &graph,
        .slog         = &slog,
    };

    /* ---- 7. Generator context ---- */
    GeneratorCtx gen = {
        .clients      = all,
        .client_count = 6,
        .workers      = workers,
        .worker_count = 3,
        .slog         = &slog,
    };

    /* ---- 8. START ALL THREADS ---- */
    printf("\n[Sim] Starting threads for %d seconds...\n\n", SIM_DURATION_S);

    relay_start(&relay);
    for (int i = 0; i < 3; i++) ap_worker_start(&workers[i]);
    generator_start(&gen);

    /* ---- 9. Let it run ---- */
    sleep(SIM_DURATION_S);

    /* ---- 10. STOP ALL THREADS (in reverse order) ---- */
    printf("\n[Sim] Stopping threads...\n");
    generator_stop(&gen);
    for (int i = 0; i < 3; i++) ap_worker_stop(&workers[i]);
    relay_stop(&relay);

    /* ---- 11. Print log + stats ---- */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  Thread-safe Event Log                               ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    slog_print(&slog);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  Final Stats                                         ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    printf("  Relay packets forwarded : %llu\n",
           (unsigned long long)atomic_load(&relay.packets_relayed));
    printf("\n  %-4s %-22s %8s %8s\n", "AP", "SSID", "TX", "Dropped");
    printf("  %s\n","--------------------------------------------");
    for (int i = 0; i < 3; i++) {
        printf("  AP%d  %-22s %8llu %8llu\n",
               i, workers[i].ap->ssid,
               (unsigned long long)atomic_load(&workers[i].packets_tx),
               (unsigned long long)atomic_load(&workers[i].packets_dropped));
    }
    printf("\n  %-20s %12s %12s\n", "Client", "Bytes Sent", "Bytes Dropped");
    printf("  %s\n","--------------------------------------------");
    for (int i = 0; i < 6; i++) {
        printf("  %-20s %12llu %12llu\n",
               all[i]->name,
               (unsigned long long)all[i]->bytes_sent,
               (unsigned long long)all[i]->bytes_dropped);
    }

    /* ---- 12. Cleanup ---- */
    sq_destroy(&outbound);
    rt_free(&route);
    ct_free(&ct);
    slog_free(&slog);

    printf("\nThreaded simulation complete. All memory freed.\n\n");
}
