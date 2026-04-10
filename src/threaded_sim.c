/* threaded_sim.c
 * Multi-threaded simulation with mutex, semaphore, atomic, AND
 * condition variables (pthread_cond).
 *
 * ─────────────────────────────────────────────────────────────────
 * SYNC PRIMITIVES USED AND WHY
 * ─────────────────────────────────────────────────────────────────
 *
 * pthread_mutex   — mutual exclusion on shared data structures
 *                   (scheduler heap, client tx_queue, log).
 *
 * sem_t           — counting semaphore in SharedQueue for bounded
 *                   producer/consumer between AP workers and relay.
 *
 * atomic_int/uint — lock-free stop flag and packet sequence counter.
 *
 * pthread_cond    — event notification so threads sleep with zero
 *                   CPU instead of polling on a timer.
 *
 *   work_ready  (on APWorker):
 *     AP worker sleeps here when its scheduler heap is empty.
 *     Generator signals it after inserting a client into the heap.
 *     Shutdown broadcasts to wake all sleeping AP workers.
 *
 *   relay_cond  (on RelayCtx):
 *     Relay thread sleeps here when all outbound queues are empty.
 *     AP worker signals it after producing a packet to outbound.
 *     Shutdown broadcasts to wake the sleeping relay thread.
 *
 * ─────────────────────────────────────────────────────────────────
 * THREAD INTERACTION DIAGRAM
 * ─────────────────────────────────────────────────────────────────
 *
 *  [GeneratorThread]
 *      loop:
 *        pick random client
 *        lock client->tx_mutex
 *          pq_enqueue(client->tx_queue, pkt)
 *        unlock client->tx_mutex
 *
 *        lock worker->sched_mutex
 *          if client not in heap: sched_insert()
 *          pthread_cond_signal(&worker->work_ready)  ← wake AP thread
 *        unlock worker->sched_mutex
 *        sleep GEN_TICK_US
 *
 *  [APWorker Thread] × N
 *      loop:
 *        lock sched_mutex
 *          while heap empty AND not stopping:
 *            pthread_cond_wait(&work_ready, &sched_mutex)  ← sleep
 *          if stopping: unlock + exit
 *          winner = sched_pop_best()
 *          lock winner->tx_mutex
 *            pq_dequeue(winner->tx_queue, &pkt)
 *            if queue still non-empty: re-insert winner into heap
 *          unlock winner->tx_mutex
 *        unlock sched_mutex
 *        sq_try_produce(outbound, pkt)
 *        lock relay->relay_mutex
 *          pthread_cond_signal(&relay->relay_cond)  ← wake relay
 *        unlock relay->relay_mutex
 *
 *  [RelayThread]
 *      loop:
 *        lock relay_mutex
 *          pthread_cond_wait(&relay_cond, &relay_mutex)  ← sleep
 *        unlock relay_mutex
 *        drain all outbound queues (sq_try_consume loop)
 *        log + BFS roaming decision per packet
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

        /*
         * CONDITION VARIABLE WAIT — canonical pattern:
         *
         *   while (condition_not_met)          ← always a while, not if
         *       pthread_cond_wait(&cond, &mtx) ← atomically: sleep + release mutex
         *
         * The while loop guards against spurious wakeups — POSIX allows
         * pthread_cond_wait to return even when nobody called signal.
         * We also check stop inside the loop so shutdown wakes us cleanly.
         */
        while (w->scheduler.size == 0 && !atomic_load(&w->stop)) {
            pthread_cond_wait(&w->work_ready, &w->sched_mutex);
        }

        /* Woken — check if we're shutting down */
        if (atomic_load(&w->stop) && w->scheduler.size == 0) {
            pthread_mutex_unlock(&w->sched_mutex);
            break;
        }

        /* Pop the best client while holding sched_mutex */
        Client *winner = sched_pop_best(&w->scheduler);
        pthread_mutex_unlock(&w->sched_mutex);

        if (!winner) continue;

        /* Dequeue one packet under the client's own mutex */
        Packet pkt;
        int got_pkt = 0;

        pthread_mutex_lock(&winner->tx_mutex);
        if (!pq_is_empty(&winner->tx_queue)) {
            pq_dequeue(&winner->tx_queue, &pkt);
            got_pkt = 1;
        }
        int has_more = !pq_is_empty(&winner->tx_queue);
        pthread_mutex_unlock(&winner->tx_mutex);

        /* Re-insert into heap if more packets remain */
        if (has_more) {
            pthread_mutex_lock(&w->sched_mutex);
            sched_insert(&w->scheduler, winner);
            pthread_mutex_unlock(&w->sched_mutex);
        }

        if (!got_pkt) continue;

        pkt.ap_id = w->ap_id;

        /* Forward to relay via SharedQueue */
        if (sq_try_produce(w->outbound, &pkt) == 0) {
            atomic_fetch_add(&w->packets_tx, 1);
            winner->bytes_sent += pkt.len;

            /*
             * Signal the relay thread that a packet is waiting.
             * We need relay_mutex only for the condvar call itself —
             * the SharedQueue's own semaphore already tracks data;
             * relay_cond is purely for waking the sleeping relay thread.
             */
            pthread_mutex_lock(&w->outbound->relay_mutex);
            pthread_cond_signal(&w->outbound->relay_cond);
            pthread_mutex_unlock(&w->outbound->relay_mutex);

        } else {
            atomic_fetch_add(&w->packets_dropped, 1);
            winner->bytes_dropped += pkt.len;
            mac_to_str(winner->mac, mac_s);
            slog_append(w->slog,
                "[AP%d] relay queue full — dropped seq=%u from %s",
                w->ap_id, pkt.seq, mac_s);
        }
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
    w->ap_id    = ap_id;
    w->ap       = ap;
    w->outbound = outbound;
    w->slog     = slog;
    atomic_store(&w->stop, 0);
    atomic_store(&w->packets_tx, 0);
    atomic_store(&w->packets_dropped, 0);
    pthread_mutex_init(&w->sched_mutex, NULL);
    pthread_cond_init(&w->work_ready, NULL);   /* <-- init condvar */
    sched_init(&w->scheduler);
}

void ap_worker_add_client(APWorker *w, Client *c) {
    if (w->client_count >= MAX_CLIENTS) return;
    w->clients[w->client_count++] = c;
    /* Pre-load into scheduler — but no packets yet, so don't signal */
    pthread_mutex_lock(&w->sched_mutex);
    sched_insert(&w->scheduler, c);
    pthread_mutex_unlock(&w->sched_mutex);
}

void ap_worker_start(APWorker *w) {
    pthread_create(&w->tid, NULL, ap_worker_thread, w);
}

void ap_worker_stop(APWorker *w) {
    /* Set stop flag, then broadcast to wake the worker even if heap is empty */
    atomic_store(&w->stop, 1);

    pthread_mutex_lock(&w->sched_mutex);
    pthread_cond_broadcast(&w->work_ready);  /* wake sleeping worker */
    pthread_mutex_unlock(&w->sched_mutex);

    pthread_join(w->tid, NULL);
    pthread_cond_destroy(&w->work_ready);
    pthread_mutex_destroy(&w->sched_mutex);
}

/* ================================================================
 * PACKET GENERATOR THREAD
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

static uint8_t ap_bcast_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

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
        memcpy(pkt.dst_mac, ap_bcast_mac, 6);
        pkt.len = (uint16_t)strlen(payload);
        memcpy(pkt.data, payload, pkt.len);
        pkt.timestamp_ms = now_ms();
        pkt.ap_id = c->ap_id;

        /* Enqueue packet under client TX mutex */
        pthread_mutex_lock(&c->tx_mutex);
        pq_enqueue(&c->tx_queue, &pkt);
        pthread_mutex_unlock(&c->tx_mutex);

        /*
         * Insert client into AP heap and SIGNAL the AP worker.
         *
         * We hold sched_mutex across both the heap insert and the
         * signal so there is no race:
         *
         *   Without this ordering, a context switch could happen
         *   between the heap insert and the signal. If the AP worker
         *   checks the heap, finds it empty, then we signal — the
         *   signal is lost and the worker sleeps forever.
         *
         *   Holding the mutex until after the signal ensures the
         *   worker either:
         *     (a) hasn't entered the wait yet → it will see the
         *         non-empty heap on next check, or
         *     (b) is already in pthread_cond_wait → the signal
         *         will wake it correctly.
         */
        APWorker *w = &g->workers[c->ap_id];
        pthread_mutex_lock(&w->sched_mutex);
        int found = 0;
        for (int i = 0; i < w->scheduler.size; i++) {
            if (w->scheduler.clients[i] == c) { found = 1; break; }
        }
        if (!found) sched_insert(&w->scheduler, c);
        pthread_cond_signal(&w->work_ready);   /* wake the AP worker */
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
 * ================================================================ */

static void *relay_thread(void *arg) {
    RelayCtx *r = (RelayCtx *)arg;
    slog_append(r->slog, "[RELAY] relay thread started");

    while (1) {
        /*
         * Wait for an AP worker to signal that a packet is ready.
         *
         * pthread_cond_wait(&relay_cond, &relay_mutex):
         *   - atomically releases relay_mutex and suspends the thread
         *   - when signalled: reacquires relay_mutex and returns
         *
         * The while loop handles spurious wakeups and the case where
         * the stop flag is set — we drain any remaining packets first.
         */
        /* All workers share a single outbound queue whose condvar we wait on */
        SharedQueue *sq = r->workers[0].outbound;
        pthread_mutex_lock(&sq->relay_mutex);
        while (!atomic_load(&r->stop)) {
            /* Check if any packets are waiting before sleeping */
            pthread_mutex_lock(&sq->mutex);
            int any = sq->count > 0;
            pthread_mutex_unlock(&sq->mutex);
            if (any) break;
            pthread_cond_wait(&sq->relay_cond, &sq->relay_mutex);
        }
        int stopping = atomic_load(&r->stop);
        pthread_mutex_unlock(&sq->relay_mutex);

        /* Drain all outbound queues */
        int drained_any = 0;
        for (int i = 0; i < r->worker_count; i++) {
            Packet pkt;
            while (sq_try_consume(&r->workers[i].outbound[0], &pkt) == 0) {
                drained_any = 1;
                atomic_fetch_add(&r->packets_relayed, 1);

                /* BFS roaming: find least-interfering neighbour AP */
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
                if (best_nb >= 0) {
                    slog_append(r->slog,
                        "[RELAY] seq=%-5u AP%d->AP%d (interf=%d%%) src=%s '%s'",
                        pkt.seq, pkt.ap_id, best_nb, best_pct,
                        src, pkt.data);
                } else {
                    slog_append(r->slog,
                        "[RELAY] seq=%-5u AP%d (no neighbours) src=%s",
                        pkt.seq, pkt.ap_id, src);
                }
            }
        }
        (void)drained_any;

        if (stopping) break;
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
    /* Broadcast on the shared outbound condvar to wake the relay thread */
    SharedQueue *sq = r->workers[0].outbound;
    pthread_mutex_lock(&sq->relay_mutex);
    pthread_cond_broadcast(&sq->relay_cond);
    pthread_mutex_unlock(&sq->relay_mutex);
    pthread_join(r->tid, NULL);
}

/* ================================================================
 * sim_run_threaded
 * ================================================================ */

void sim_run_threaded(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║         THREADED SIMULATION                          ║\n");
    printf("║  Threads: 1 generator + N AP workers + 1 relay       ║\n");
    printf("║  Sync:    mutex · semaphore · atomic · cond_var       ║\n");
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
    Client *all[6] = {c0,c1,c2,c3,c4,c5};

    ct_insert(&ct,c0); ct_insert(&ct,c1); ct_insert(&ct,c2);
    ct_insert(&ct,c3); ct_insert(&ct,c4); ct_insert(&ct,c5);

    ap_worker_add_client(&workers[id0], c0);
    ap_worker_add_client(&workers[id0], c1);
    ap_worker_add_client(&workers[id0], c4);
    ap_worker_add_client(&workers[id1], c2);
    ap_worker_add_client(&workers[id2], c3);
    ap_worker_add_client(&workers[id2], c5);

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

    GeneratorCtx gen = {
        .clients      = all,
        .client_count = 6,
        .workers      = workers,
        .worker_count = 3,
        .slog         = &slog,
    };

    printf("\n[Sim] Starting threads for %d seconds...\n", SIM_DURATION_S);
    printf("[Sync] AP workers sleep on work_ready condvar (no busy-wait)\n");
    printf("[Sync] Relay thread sleeps on relay_cond condvar (no busy-wait)\n\n");

    relay_start(&relay);
    for (int i = 0; i < 3; i++) ap_worker_start(&workers[i]);
    generator_start(&gen);

    sleep(SIM_DURATION_S);

    printf("\n[Sim] Stopping — broadcasting to all condvars...\n");
    generator_stop(&gen);
    for (int i = 0; i < 3; i++) ap_worker_stop(&workers[i]);
    relay_stop(&relay);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  Thread-safe Event Log                               ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    slog_print(&slog);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  Final Stats                                         ║\n");
    printf("╚══════════════════════════════════════════════════════╝\n");
    printf("  Packets relayed : %llu\n",
           (unsigned long long)atomic_load(&relay.packets_relayed));
    printf("\n  %-4s %-22s %8s %8s\n","AP","SSID","TX","Dropped");
    printf("  %s\n","--------------------------------------------");
    for (int i = 0; i < 3; i++) {
        printf("  AP%d  %-22s %8llu %8llu\n", i, workers[i].ap->ssid,
               (unsigned long long)atomic_load(&workers[i].packets_tx),
               (unsigned long long)atomic_load(&workers[i].packets_dropped));
    }
    printf("\n  %-20s %12s %12s\n","Client","Bytes Sent","Bytes Dropped");
    printf("  %s\n","--------------------------------------------");
    for (int i = 0; i < 6; i++) {
        printf("  %-20s %12llu %12llu\n", all[i]->name,
               (unsigned long long)all[i]->bytes_sent,
               (unsigned long long)all[i]->bytes_dropped);
    }

    sq_destroy(&outbound);
    rt_free(&route);
    ct_free(&ct);
    slog_free(&slog);
    printf("\nThreaded simulation complete. All memory freed.\n\n");
}
