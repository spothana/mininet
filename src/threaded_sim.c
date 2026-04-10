/* threaded_sim.c
 *
 * Full duplex simulation — TX and RX paths.
 *
 * ─────────────────────────────────────────────────────────
 *  TX PATH  (uplink: client device → network)
 * ─────────────────────────────────────────────────────────
 *
 *  [GeneratorThread]
 *    Simulates client devices producing frames.
 *    lock client->tx_mutex
 *      pq_enqueue(client->tx_queue)        ← software TX FIFO
 *    unlock
 *    lock worker->sched_mutex
 *      sched_insert() + cond_signal(work_ready)
 *    unlock
 *
 *  [APWorker Thread × N]
 *    Woken by work_ready condvar.
 *    sched_pop_best() → winner client
 *    lock winner->tx_mutex
 *      pq_dequeue(winner->tx_queue)
 *    unlock
 *    sq_try_produce(outbound)              ← TX SharedQueue → relay
 *    cond_signal(relay_cond)
 *
 *  [RelayThread]
 *    Woken by relay_cond condvar.
 *    sq_try_consume(outbound)
 *    BFS roaming decision on AP graph
 *    sq_produce(target_ap->rx_queue)      ← injects into RX path
 *    cond_signal(rx_ready condvar on rx_queue)
 *
 * ─────────────────────────────────────────────────────────
 *  RX PATH  (downlink: network → client device)
 * ─────────────────────────────────────────────────────────
 *
 *  [RelayThread]  (continues from TX path above)
 *    After BFS: pushes frame into target APWorker->rx_queue
 *    (simulates the AP receiving a frame off the air for a
 *     locally associated client — like 802.11 AP-mode RX)
 *
 *  [ReceiverThread]
 *    Drains each APWorker->rx_queue.
 *    For each frame:
 *      Look up dst_mac in ClientTable    ← O(1) hash lookup
 *      lock client->rx_mutex
 *        rb_push(client->rx_ring)        ← DMA RX descriptor ring
 *        pthread_cond_signal(client->rx_ready)
 *      unlock
 *      update bytes_received, packets_rx
 *
 *    The rx_ready condvar is the correct hook for a NAPI-style
 *    consumer: any thread waiting on frames does:
 *      while (rb_is_empty(&c->rx_ring))
 *          pthread_cond_wait(&c->rx_ready, &c->rx_mutex)
 *      rb_pop(&c->rx_ring, &pkt)
 *
 * ─────────────────────────────────────────────────────────
 *  SYNC PRIMITIVES SUMMARY
 * ─────────────────────────────────────────────────────────
 *  work_ready  pthread_cond  Gen   → AP worker  (TX work available)
 *  relay_cond  pthread_cond  AP    → Relay       (TX frame ready)
 *  rx_ready    pthread_cond  Recv  → consumer    (RX frame delivered)
 *  tx_mutex    pthread_mutex per-client TX queue / ring
 *  rx_mutex    pthread_mutex per-client RX ring
 *  sched_mutex pthread_mutex per-AP scheduler heap
 *  sq.mutex    pthread_mutex SharedQueue head/tail
 *  sem_slots   sem_t         SharedQueue flow-control (producer)
 *  sem_items   sem_t         SharedQueue flow-control (consumer)
 *  seq_counter atomic_uint   lock-free packet sequencing
 *  stop        atomic_int    clean shutdown signal
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

    while (1) {
        pthread_mutex_lock(&w->sched_mutex);
        while (w->scheduler.size == 0 && !atomic_load(&w->stop))
            pthread_cond_wait(&w->work_ready, &w->sched_mutex);

        if (atomic_load(&w->stop) && w->scheduler.size == 0) {
            pthread_mutex_unlock(&w->sched_mutex);
            break;
        }

        Client *winner = sched_pop_best(&w->scheduler);
        pthread_mutex_unlock(&w->sched_mutex);
        if (!winner) continue;

        Packet pkt;
        int got_pkt = 0;

        pthread_mutex_lock(&winner->tx_mutex);
        if (!pq_is_empty(&winner->tx_queue)) {
            pq_dequeue(&winner->tx_queue, &pkt);
            got_pkt = 1;
        }
        int has_more = !pq_is_empty(&winner->tx_queue);
        pthread_mutex_unlock(&winner->tx_mutex);

        if (has_more) {
            pthread_mutex_lock(&w->sched_mutex);
            sched_insert(&w->scheduler, winner);
            pthread_mutex_unlock(&w->sched_mutex);
        }

        if (!got_pkt) continue;

        pkt.ap_id = w->ap_id;

        /* TX: forward to relay via outbound SharedQueue */
        if (sq_try_produce(w->outbound, &pkt) == 0) {
            atomic_fetch_add(&w->packets_tx, 1);
            winner->bytes_sent += pkt.len;

            pthread_mutex_lock(&w->outbound->relay_mutex);
            pthread_cond_signal(&w->outbound->relay_cond);
            pthread_mutex_unlock(&w->outbound->relay_mutex);
        } else {
            atomic_fetch_add(&w->packets_dropped, 1);
            winner->bytes_dropped += pkt.len;
            mac_to_str(winner->mac, mac_s);
            slog_append(w->slog,
                "[AP%d] TX relay full — dropped seq=%u from %s",
                w->ap_id, pkt.seq, mac_s);
        }
    }

    slog_append(w->slog,
        "[AP%d '%s'] stopping. tx=%llu rx_queued=%llu dropped=%llu",
        w->ap_id, w->ap->ssid,
        (unsigned long long)atomic_load(&w->packets_tx),
        (unsigned long long)atomic_load(&w->packets_rx),
        (unsigned long long)atomic_load(&w->packets_dropped));
    return NULL;
}

void ap_worker_init(APWorker *w, int ap_id, AccessPoint *ap,
                    SharedQueue *outbound, SafeLog *slog) {
    memset(w, 0, sizeof(APWorker));
    w->ap_id    = ap_id;
    w->ap       = ap;
    w->outbound = outbound;
    w->slog     = slog;
    atomic_store(&w->stop, 0);
    atomic_store(&w->packets_tx, 0);
    atomic_store(&w->packets_rx, 0);
    atomic_store(&w->packets_dropped, 0);
    pthread_mutex_init(&w->sched_mutex, NULL);
    pthread_cond_init(&w->work_ready, NULL);
    sched_init(&w->scheduler);
    sq_init(&w->rx_queue);   /* RX path: relay → receiver thread */
}

void ap_worker_add_client(APWorker *w, Client *c) {
    if (w->client_count >= MAX_CLIENTS) return;
    w->clients[w->client_count++] = c;
    pthread_mutex_lock(&w->sched_mutex);
    sched_insert(&w->scheduler, c);
    pthread_mutex_unlock(&w->sched_mutex);
}

void ap_worker_start(APWorker *w) {
    pthread_create(&w->tid, NULL, ap_worker_thread, w);
}

void ap_worker_stop(APWorker *w) {
    atomic_store(&w->stop, 1);
    pthread_mutex_lock(&w->sched_mutex);
    pthread_cond_broadcast(&w->work_ready);
    pthread_mutex_unlock(&w->sched_mutex);
    pthread_join(w->tid, NULL);
    pthread_cond_destroy(&w->work_ready);
    pthread_mutex_destroy(&w->sched_mutex);
    sq_destroy(&w->rx_queue);
}

/* ================================================================
 * PACKET GENERATOR THREAD  (TX uplink producer)
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

static uint8_t bcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static void *generator_thread(void *arg) {
    GeneratorCtx *g = (GeneratorCtx *)arg;
    slog_append(g->slog, "[GEN] generator thread started");

    while (!atomic_load(&g->stop)) {
        int idx = (int)(now_ms() % (uint64_t)g->client_count);
        Client *c = g->clients[idx];

        uint32_t seq = atomic_fetch_add(&g->seq_counter, 1);
        const char *payload = sample_payloads[seq % N_PAYLOADS];

        Packet pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.seq = seq;
        memcpy(pkt.src_mac, c->mac, 6);
        memcpy(pkt.dst_mac, bcast_mac, 6);
        pkt.len = (uint16_t)strlen(payload);
        memcpy(pkt.data, payload, pkt.len);
        pkt.timestamp_ms = now_ms();
        pkt.ap_id = c->ap_id;

        pthread_mutex_lock(&c->tx_mutex);
        pq_enqueue(&c->tx_queue, &pkt);
        pthread_mutex_unlock(&c->tx_mutex);

        APWorker *w = &g->workers[c->ap_id];
        pthread_mutex_lock(&w->sched_mutex);
        int found = 0;
        for (int i = 0; i < w->scheduler.size; i++)
            if (w->scheduler.clients[i] == c) { found = 1; break; }
        if (!found) sched_insert(&w->scheduler, c);
        pthread_cond_signal(&w->work_ready);
        pthread_mutex_unlock(&w->sched_mutex);

        sleep_us(GEN_TICK_US);
    }

    slog_append(g->slog, "[GEN] stopping. seq_hi=%u",
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
 * RELAY THREAD
 * Drains outbound (TX) queue, does BFS roaming decision,
 * then injects the frame into the target AP's rx_queue (RX path).
 * ================================================================ */

static void *relay_thread(void *arg) {
    RelayCtx *r = (RelayCtx *)arg;
    slog_append(r->slog, "[RELAY] relay thread started");

    while (1) {
        SharedQueue *sq = r->workers[0].outbound;
        pthread_mutex_lock(&sq->relay_mutex);
        while (!atomic_load(&r->stop)) {
            pthread_mutex_lock(&sq->mutex);
            int any = sq->count > 0;
            pthread_mutex_unlock(&sq->mutex);
            if (any) break;
            pthread_cond_wait(&sq->relay_cond, &sq->relay_mutex);
        }
        int stopping = atomic_load(&r->stop);
        pthread_mutex_unlock(&sq->relay_mutex);

        for (int i = 0; i < r->worker_count; i++) {
            Packet pkt;
            while (sq_try_consume(&r->workers[i].outbound[0], &pkt) == 0) {
                atomic_fetch_add(&r->packets_relayed, 1);

                /* BFS: find least-interfering neighbour AP */
                const AccessPoint *ap = &r->graph->aps[pkt.ap_id];
                int best_nb  = -1;
                int best_pct = 101;
                for (int e = 0; e < ap->neighbor_count; e++) {
                    if (ap->neighbors[e].interference_pct < best_pct) {
                        best_pct = ap->neighbors[e].interference_pct;
                        best_nb  = ap->neighbors[e].neighbor_id;
                    }
                }

                char src[18];
                mac_to_str(pkt.src_mac, src);

                /* ── TX log ── */
                if (best_nb >= 0)
                    slog_append(r->slog,
                        "[TX] seq=%-5u AP%d->AP%d (%d%%) src=%s '%s'",
                        pkt.seq, pkt.ap_id, best_nb, best_pct,
                        src, pkt.data);
                else
                    slog_append(r->slog,
                        "[TX] seq=%-5u AP%d (no neighbours) src=%s",
                        pkt.seq, pkt.ap_id, src);

                /*
                 * ── RX inject ──
                 * Build a downlink frame: src becomes the AP bcast MAC,
                 * dst is the original sender (echo / ACK simulation).
                 * Push into the target AP's rx_queue so the receiver
                 * thread can dispatch it to the right client's rx_ring.
                 *
                 * Target AP: use best_nb if found, otherwise same AP.
                 */
                int target_ap = (best_nb >= 0) ? best_nb : pkt.ap_id;
                Packet rx_pkt = pkt;
                memcpy(rx_pkt.dst_mac, pkt.src_mac, 6);
                memcpy(rx_pkt.src_mac, bcast_mac,   6);
                rx_pkt.ap_id = target_ap;
                rx_pkt.seq  |= 0x80000000u; /* high bit marks RX frames */

                APWorker *target = &r->workers[target_ap];
                if (sq_try_produce(&target->rx_queue, &rx_pkt) == 0) {
                    atomic_fetch_add(&target->packets_rx, 1);
                    /* signal receiver thread via rx_queue's relay_cond */
                    pthread_mutex_lock(&target->rx_queue.relay_mutex);
                    pthread_cond_signal(&target->rx_queue.relay_cond);
                    pthread_mutex_unlock(&target->rx_queue.relay_mutex);
                } else {
                    slog_append(r->slog,
                        "[RX] AP%d rx_queue full — dropped downlink seq=%u",
                        target_ap, rx_pkt.seq);
                }
            }
        }

        if (stopping) break;
    }

    slog_append(r->slog, "[RELAY] stopping. relayed=%llu",
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
    SharedQueue *sq = r->workers[0].outbound;
    pthread_mutex_lock(&sq->relay_mutex);
    pthread_cond_broadcast(&sq->relay_cond);
    pthread_mutex_unlock(&sq->relay_mutex);
    pthread_join(r->tid, NULL);
}

/* ================================================================
 * RECEIVER THREAD  (RX downlink dispatcher)
 *
 * Drains each APWorker->rx_queue.
 * Looks up dst_mac in the ClientTable (hash — O(1)).
 * Pushes the frame into the matching client's rx_ring (DMA ring).
 * Signals client->rx_ready condvar so any consumer can wake.
 * ================================================================ */

static void *receiver_thread(void *arg) {
    ReceiverCtx *rx = (ReceiverCtx *)arg;
    slog_append(rx->slog, "[RECV] receiver thread started");

    while (1) {
        /*
         * Wait for any AP's rx_queue to have frames.
         * We use workers[0].rx_queue's relay_cond as the single
         * wake-up signal; the loop below drains ALL AP rx_queues
         * each time it wakes, so using one condvar is sufficient.
         */
        SharedQueue *wake_sq = &rx->workers[0].rx_queue;
        pthread_mutex_lock(&wake_sq->relay_mutex);
        while (!atomic_load(&rx->stop)) {
            int any = 0;
            for (int i = 0; i < rx->worker_count && !any; i++) {
                pthread_mutex_lock(&rx->workers[i].rx_queue.mutex);
                any = rx->workers[i].rx_queue.count > 0;
                pthread_mutex_unlock(&rx->workers[i].rx_queue.mutex);
            }
            if (any) break;
            pthread_cond_wait(&wake_sq->relay_cond, &wake_sq->relay_mutex);
        }
        int stopping = atomic_load(&rx->stop);
        pthread_mutex_unlock(&wake_sq->relay_mutex);

        /* Drain all AP rx_queues */
        for (int i = 0; i < rx->worker_count; i++) {
            Packet pkt;
            while (sq_try_consume(&rx->workers[i].rx_queue, &pkt) == 0) {

                /* Hash lookup: dst_mac → Client* */
                Client *c = ct_find(rx->ct, pkt.dst_mac);
                if (!c) {
                    atomic_fetch_add(&rx->packets_unknown, 1);
                    char dst[18];
                    mac_to_str(pkt.dst_mac, dst);
                    slog_append(rx->slog,
                        "[RECV] unknown dst %s — dropped seq=%u",
                        dst, pkt.seq);
                    continue;
                }

                /*
                 * Push into client's rx_ring (DMA-style receive ring).
                 * rx_ring is a fixed circular buffer — no malloc.
                 * rx_mutex protects head/tail/count.
                 * rx_ready condvar wakes any consumer waiting on frames.
                 */
                pthread_mutex_lock(&c->rx_mutex);
                int pushed = rb_push(&c->rx_ring, &pkt);
                if (pushed == 0) {
                    c->bytes_received += pkt.len;
                    c->packets_rx++;
                    /* signal any waiting consumer (NAPI-style) */
                    pthread_cond_signal(&c->rx_ready);
                    char dst[18];
                    mac_to_str(pkt.dst_mac, dst);
                    slog_append(rx->slog,
                        "[RECV] %-20s rx_ring ← seq=%u AP%d '%s'",
                        c->name, pkt.seq & 0x7FFFFFFFu,
                        i, pkt.data);
                } else {
                    /* rx_ring full — frame dropped at client */
                    c->bytes_dropped += pkt.len;
                    slog_append(rx->slog,
                        "[RECV] %s rx_ring FULL — dropped seq=%u",
                        c->name, pkt.seq & 0x7FFFFFFFu);
                }
                pthread_mutex_unlock(&c->rx_mutex);

                atomic_fetch_add(&rx->packets_dispatched, 1);
            }
        }

        if (stopping) break;
    }

    slog_append(rx->slog, "[RECV] stopping. dispatched=%llu unknown=%llu",
                (unsigned long long)atomic_load(&rx->packets_dispatched),
                (unsigned long long)atomic_load(&rx->packets_unknown));
    return NULL;
}

void receiver_start(ReceiverCtx *rx) {
    atomic_store(&rx->stop, 0);
    atomic_store(&rx->packets_dispatched, 0);
    atomic_store(&rx->packets_unknown, 0);
    pthread_create(&rx->tid, NULL, receiver_thread, rx);
}

void receiver_stop(ReceiverCtx *rx) {
    atomic_store(&rx->stop, 1);
    /* broadcast on all AP rx_queue condvars to wake receiver */
    for (int i = 0; i < rx->worker_count; i++) {
        pthread_mutex_lock(&rx->workers[i].rx_queue.relay_mutex);
        pthread_cond_broadcast(&rx->workers[i].rx_queue.relay_cond);
        pthread_mutex_unlock(&rx->workers[i].rx_queue.relay_mutex);
    }
    pthread_join(rx->tid, NULL);
}

/* ================================================================
 * sim_run_threaded  —  full-duplex wiring
 * ================================================================ */

void sim_run_threaded(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  THREADED SIMULATION — full duplex TX + RX           ║\n");
    printf("║  Threads: generator · AP workers · relay · receiver  ║\n");
    printf("║  Sync:    mutex · sem · atomic · cond_var            ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");

    SafeLog     slog;
    APGraph     graph;
    RouteTable  route;
    SharedQueue outbound;
    ClientTable ct;

    slog_init(&slog);
    graph_init(&graph);
    rt_init(&route);
    sq_init(&outbound);
    ct_init(&ct);

    int id0 = graph_add_ap(&graph, "FordEVDD-Main",  BAND_5G, 36, 20);
    int id1 = graph_add_ap(&graph, "FordEVDD-North", BAND_5G, 40, 18);
    int id2 = graph_add_ap(&graph, "FordEVDD-South", BAND_2G,  6, 15);
    graph_add_edge(&graph, id0, id1, 30);
    graph_add_edge(&graph, id0, id2, 10);
    graph_add_edge(&graph, id1, id2, 20);

    printf("\n[Setup] AP graph:\n");
    graph_print(&graph);

    APWorker workers[3];
    ap_worker_init(&workers[0], id0, &graph.aps[id0], &outbound, &slog);
    ap_worker_init(&workers[1], id1, &graph.aps[id1], &outbound, &slog);
    ap_worker_init(&workers[2], id2, &graph.aps[id2], &outbound, &slog);

    uint8_t mac0[6]={0xAA,0xBB,0xCC,0x11,0x22,0x33};
    uint8_t mac1[6]={0xAA,0xBB,0xCC,0x44,0x55,0x66};
    uint8_t mac2[6]={0xDE,0xAD,0xBE,0xEF,0x00,0x01};
    uint8_t mac3[6]={0xDE,0xAD,0xBE,0xEF,0x00,0x02};
    uint8_t mac4[6]={0xCA,0xFE,0xBA,0xBE,0x00,0x01};
    uint8_t mac5[6]={0x10,0x20,0x30,0x40,0x50,0x60};

    Client *c0=client_new(mac0,"192.168.1.10","iPhone-CarPlay",   -42,9,BAND_5G,36,id0);
    Client *c1=client_new(mac1,"192.168.1.11","Pixel-AndroidAuto",-48,8,BAND_5G,36,id0);
    Client *c2=client_new(mac2,"192.168.1.20","iPad-Tablet",      -61,5,BAND_5G,40,id1);
    Client *c3=client_new(mac3,"192.168.1.21","Samsung-Phone",    -72,4,BAND_2G, 6,id2);
    Client *c4=client_new(mac4,"192.168.1.30","MacBook-Laptop",   -55,6,BAND_5G,36,id0);
    Client *c5=client_new(mac5,"192.168.1.40","ESP32-IoT",        -80,1,BAND_2G, 6,id2);
    Client *all[6]={c0,c1,c2,c3,c4,c5};

    ct_insert(&ct,c0); ct_insert(&ct,c1); ct_insert(&ct,c2);
    ct_insert(&ct,c3); ct_insert(&ct,c4); ct_insert(&ct,c5);

    ap_worker_add_client(&workers[id0],c0);
    ap_worker_add_client(&workers[id0],c1);
    ap_worker_add_client(&workers[id0],c4);
    ap_worker_add_client(&workers[id1],c2);
    ap_worker_add_client(&workers[id2],c3);
    ap_worker_add_client(&workers[id2],c5);

    printf("\n[Setup] Clients:\n");
    ct_print(&ct);

    rt_insert(&route,"192.168.1.10",mac0); rt_insert(&route,"192.168.1.11",mac1);
    rt_insert(&route,"192.168.1.20",mac2); rt_insert(&route,"192.168.1.21",mac3);
    rt_insert(&route,"192.168.1.30",mac4); rt_insert(&route,"192.168.1.40",mac5);
    printf("\n[Setup] Routing table:\n");
    rt_print(&route);

    RelayCtx relay = {
        .workers      = workers,
        .worker_count = 3,
        .graph        = &graph,
        .slog         = &slog,
    };

    ReceiverCtx recv = {
        .workers      = workers,
        .worker_count = 3,
        .ct           = &ct,
        .slog         = &slog,
    };

    GeneratorCtx gen = {
        .clients      = all,
        .client_count = 6,
        .workers      = workers,
        .worker_count = 3,
        .slog         = &slog,
    };

    printf("\n[Sim] Starting threads for %d seconds...\n", SIM_DURATION_S);
    printf("[TX]  Generator → AP worker → Relay → rx_queue\n");
    printf("[RX]  Relay → rx_queue → Receiver → client rx_ring\n\n");

    relay_start(&relay);
    receiver_start(&recv);
    for (int i = 0; i < 3; i++) ap_worker_start(&workers[i]);
    generator_start(&gen);

    sleep(SIM_DURATION_S);

    printf("\n[Sim] Stopping all threads...\n");
    generator_stop(&gen);
    for (int i = 0; i < 3; i++) ap_worker_stop(&workers[i]);
    relay_stop(&relay);
    receiver_stop(&recv);

    /* ── Print log ── */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  Event Log                                           ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    slog_print(&slog);

    /* ── Print stats ── */
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  Final Stats                                         ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    printf("  Relay forwarded : %llu\n",
           (unsigned long long)atomic_load(&relay.packets_relayed));
    printf("  Recv dispatched : %llu   unknown dst: %llu\n",
           (unsigned long long)atomic_load(&recv.packets_dispatched),
           (unsigned long long)atomic_load(&recv.packets_unknown));

    printf("\n  %-4s %-22s %8s %8s %8s\n","AP","SSID","TX","RX-q","Dropped");
    printf("  %s\n","--------------------------------------------------");
    for (int i = 0; i < 3; i++) {
        printf("  AP%d  %-22s %8llu %8llu %8llu\n", i,
               workers[i].ap->ssid,
               (unsigned long long)atomic_load(&workers[i].packets_tx),
               (unsigned long long)atomic_load(&workers[i].packets_rx),
               (unsigned long long)atomic_load(&workers[i].packets_dropped));
    }

    printf("\n  %-20s %10s %10s %10s %8s\n",
           "Client","Bytes TX","Bytes RX","Pkts RX","rx_ring");
    printf("  %s\n","-----------------------------------------------------");
    for (int i = 0; i < 6; i++) {
        Client *c = all[i];
        /* lock rx_mutex to safely read rx_ring count */
        pthread_mutex_lock(&c->rx_mutex);
        int rx_ring_count = c->rx_ring.count;
        pthread_mutex_unlock(&c->rx_mutex);
        printf("  %-20s %10llu %10llu %10llu %8d\n",
               c->name,
               (unsigned long long)c->bytes_sent,
               (unsigned long long)c->bytes_received,
               (unsigned long long)c->packets_rx,
               rx_ring_count);
    }

    sq_destroy(&outbound);
    rt_free(&route);
    ct_free(&ct);
    slog_free(&slog);
    printf("\nFull-duplex simulation complete. All memory freed.\n\n");
}
