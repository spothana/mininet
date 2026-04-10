/* simulation.c
 * Full end-to-end simulation using every data structure in the project.
 *
 * Scenario:
 *   - 3 Access Points with RF interference edges
 *   - 6 clients on different bands / channels
 *   - Packets generated, enqueued, some dropped to retry stack
 *   - Scheduler picks clients in priority order each "tick"
 *   - Routing BST maps IPs to MACs
 *   - All events logged in the dynamic EventLog
 */
#include "mininet.h"

/* ---- helpers ---- */

static Packet make_packet(uint32_t seq,
                          const uint8_t src[6], const uint8_t dst[6],
                          const char *payload) {
    Packet p;
    memset(&p, 0, sizeof(p));
    p.seq = seq;
    memcpy(p.src_mac, src, 6);
    memcpy(p.dst_mac, dst, 6);
    p.len = (uint16_t)strlen(payload);
    if (p.len >= MAX_PACKET_DATA) p.len = MAX_PACKET_DATA - 1;
    memcpy(p.data, payload, p.len);
    p.data[p.len] = '\0';
    p.timestamp_ms = now_ms();
    return p;
}

static void section(const char *title) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════╗\n");
    printf("║  %-52s║\n", title);
    printf("╚══════════════════════════════════════════════════════╝\n");
}

/* ---- main sim ---- */

void sim_run(void) {
    EventLog   log;
    ClientTable ct;
    APGraph    graph;
    RouteNode *route_root = NULL;
    Scheduler  sched;

    log_init(&log);
    ct_init(&ct);
    graph_init(&graph);
    sched_init(&sched);

    log_append(&log, "MiniNet simulator starting up");

    /* ======================================================
     * 1. BUILD ACCESS POINT GRAPH
     * ====================================================== */
    section("1. Access Point Graph");

    int ap0 = graph_add_ap(&graph, "FordEVDD-Main",   BAND_5G, 36,  20);
    int ap1 = graph_add_ap(&graph, "FordEVDD-North",  BAND_5G, 40,  18);
    int ap2 = graph_add_ap(&graph, "FordEVDD-South",  BAND_2G,  6,  15);
    int ap3 = graph_add_ap(&graph, "FordEVDD-Lab",    BAND_6G,  1,  23);

    /* Interference edges (adjacent channels on same band interfere) */
    graph_add_edge(&graph, ap0, ap1, 30);   /* ch36 <-> ch40, mild overlap */
    graph_add_edge(&graph, ap0, ap2, 10);   /* 5G <-> 2G, cross-band low */
    graph_add_edge(&graph, ap1, ap3, 5);    /* 5G <-> 6G, very low         */
    graph_add_edge(&graph, ap2, ap3, 2);    /* 2G <-> 6G, negligible       */

    graph_print(&graph);
    log_append(&log, "AP graph built: %d APs, edges set", graph.ap_count);

    /* BFS roaming path from Main AP */
    printf("\n");
    graph_bfs(&graph, ap0);

    /* DFS coverage from Lab AP */
    int visited[MAX_APS] = {0};
    printf("\n");
    graph_dfs(&graph, ap3, visited);

    /* ======================================================
     * 2. REGISTER CLIENTS IN HASH TABLE
     * ====================================================== */
    section("2. Client Registration (Hash Table)");

    /* MAC addresses (hardcoded for determinism) */
    uint8_t mac_carplay[6]  = {0xAA,0xBB,0xCC,0x11,0x22,0x33};
    uint8_t mac_andauto[6]  = {0xAA,0xBB,0xCC,0x44,0x55,0x66};
    uint8_t mac_tablet[6]   = {0xDE,0xAD,0xBE,0xEF,0x00,0x01};
    uint8_t mac_phone[6]    = {0xDE,0xAD,0xBE,0xEF,0x00,0x02};
    uint8_t mac_laptop[6]   = {0xCA,0xFE,0xBA,0xBE,0x00,0x01};
    uint8_t mac_iot[6]      = {0x10,0x20,0x30,0x40,0x50,0x60};

    Client *cp  = client_new(mac_carplay, "192.168.1.10", "iPhone-CarPlay",    -42, 9, BAND_5G, 36);
    Client *aa  = client_new(mac_andauto, "192.168.1.11", "Pixel-AndroidAuto", -48, 8, BAND_5G, 36);
    Client *tab = client_new(mac_tablet,  "192.168.1.20", "iPad-Tablet",       -61, 5, BAND_5G, 40);
    Client *ph  = client_new(mac_phone,   "192.168.1.21", "Samsung-Phone",     -72, 4, BAND_2G,  6);
    Client *lap = client_new(mac_laptop,  "192.168.1.30", "MacBook-Laptop",    -55, 6, BAND_6G,  1);
    Client *iot = client_new(mac_iot,     "192.168.1.40", "ESP32-IoT",         -80, 1, BAND_2G,  6);

    ct_insert(&ct, cp);
    ct_insert(&ct, aa);
    ct_insert(&ct, tab);
    ct_insert(&ct, ph);
    ct_insert(&ct, lap);
    ct_insert(&ct, iot);

    ct_print(&ct);
    log_append(&log, "6 clients registered in hash table");

    /* Lookup test */
    printf("\nLookup test (Pixel-AndroidAuto by MAC): ");
    Client *found = ct_find(&ct, mac_andauto);
    printf("%s\n", found ? found->name : "NOT FOUND");

    /* ======================================================
     * 3. POPULATE ROUTING BST
     * ====================================================== */
    section("3. Routing Table (Binary Search Tree: IP -> MAC)");

    route_root = bst_insert(route_root, "192.168.1.10", mac_carplay);
    route_root = bst_insert(route_root, "192.168.1.11", mac_andauto);
    route_root = bst_insert(route_root, "192.168.1.20", mac_tablet);
    route_root = bst_insert(route_root, "192.168.1.21", mac_phone);
    route_root = bst_insert(route_root, "192.168.1.30", mac_laptop);
    route_root = bst_insert(route_root, "192.168.1.40", mac_iot);

    printf("BST in-order (sorted by IP string):\n");
    bst_inorder(route_root);

    /* Search */
    RouteNode *rn = bst_search(route_root, "192.168.1.30");
    if (rn) {
        char mac_s[18]; mac_to_str(rn->mac, mac_s);
        printf("\nRoute lookup 192.168.1.30 -> %s\n", mac_s);
    }

    /* Delete IoT device and re-print */
    route_root = bst_delete(route_root, "192.168.1.40");
    printf("After deleting 192.168.1.40:\n");
    bst_inorder(route_root);
    log_append(&log, "Routing BST built, IoT entry deleted");

    /* ======================================================
     * 4. RING BUFFER — INJECT PACKETS
     * ====================================================== */
    section("4. Ring Buffer (CarPlay TX window)");

    uint8_t ap_mac[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    for (int i = 0; i < 6; i++) {
        char payload[64];
        snprintf(payload, sizeof(payload), "CarPlay-video-frame-%d", i);
        Packet p = make_packet(1000+i, mac_carplay, ap_mac, payload);
        int rc = rb_push(&cp->tx_ring, &p);
        printf("  rb_push seq=%u  %s\n", p.seq, rc==0?"OK":"FULL(dropped)");
        if (rc != 0) log_append(&log, "Ring buffer full, dropped seq=%u", p.seq);
    }
    rb_print(&cp->tx_ring);

    /* Drain 3 frames */
    printf("\nDraining 3 frames from ring:\n");
    for (int i = 0; i < 3; i++) {
        Packet out;
        if (rb_pop(&cp->tx_ring, &out) == 0)
            printf("  rb_pop seq=%u  data='%s'\n", out.seq, out.data);
    }
    rb_print(&cp->tx_ring);

    /* ======================================================
     * 5. PACKET QUEUE — PER-CLIENT TX FIFO
     * ====================================================== */
    section("5. Packet Queue (FIFO linked list — Android Auto)");

    for (int i = 0; i < 4; i++) {
        char payload[64];
        snprintf(payload, sizeof(payload), "AndroidAuto-audio-chunk-%d", i);
        Packet p = make_packet(2000+i, mac_andauto, ap_mac, payload);
        pq_enqueue(&aa->tx_queue, &p);
        printf("  enqueued seq=%u\n", p.seq);
    }
    pq_print(&aa->tx_queue);

    printf("\nDequeue 2 packets:\n");
    for (int i = 0; i < 2; i++) {
        Packet out;
        if (pq_dequeue(&aa->tx_queue, &out) == 0)
            printf("  dequeued seq=%u  data='%s'\n", out.seq, out.data);
    }
    pq_print(&aa->tx_queue);
    log_append(&log, "AndroidAuto TX queue processed");

    /* ======================================================
     * 6. RETRY STACK — SIMULATE PACKET LOSS + BACKOFF
     * ====================================================== */
    section("6. Retry Stack (LIFO — CSMA/CA backoff)");

    /* Simulate 3 failed transmissions */
    for (int i = 0; i < 3; i++) {
        char payload[64];
        snprintf(payload, sizeof(payload), "iPad-failed-frame-%d", i);
        Packet p = make_packet(3000+i, mac_tablet, ap_mac, payload);
        p.retry_count = (uint8_t)(i + 1);
        uint32_t backoff = (uint32_t)(1 << p.retry_count) * 10; /* 20, 40, 80 ms */
        rs_push(&tab->retry_stack, &p, backoff);
        printf("  pushed seq=%u  backoff=%ums  retry#%u\n",
               p.seq, backoff, p.retry_count);
    }

    printf("\nPop and retransmit (LIFO order):\n");
    while (!rs_is_empty(&tab->retry_stack)) {
        Packet out; uint32_t bo;
        rs_pop(&tab->retry_stack, &out, &bo);
        printf("  retransmitting seq=%u  backoff=%ums  data='%s'\n",
               out.seq, bo, out.data);
        log_append(&log, "Retry seq=%u backoff=%ums", out.seq, bo);
    }

    /* ======================================================
     * 7. SCHEDULER — MIN-HEAP AIRTIME ALLOCATION
     * ====================================================== */
    section("7. Scheduler (Min-Heap: best signal / priority first)");

    /* Insert all clients */
    Client *all_clients[] = {cp, aa, tab, ph, lap, iot};
    for (int i = 0; i < 6; i++) sched_insert(&sched, all_clients[i]);

    sched_print(&sched);

    printf("\nScheduling order (5 ticks):\n");
    /* Simulate 5 ticks: pop best, grant airtime, re-insert */
    for (int tick = 0; tick < 5; tick++) {
        Client *winner = sched_pop_best(&sched);
        if (!winner) break;
        printf("  Tick %d: granted airtime -> %-20s (signal=%ddBm prio=%d)\n",
               tick+1, winner->name, winner->signal_dbm, winner->priority);
        winner->bytes_sent += 1500;  /* simulate one MTU frame sent */
        log_append(&log, "Tick %d: %s granted airtime", tick+1, winner->name);
        sched_insert(&sched, winner);  /* re-insert for next round */
    }

    /* ======================================================
     * 8. EVENT LOG — DYNAMIC ARRAY
     * ====================================================== */
    section("8. Event Log (Dynamic Resizable Array)");

    log_append(&log, "Simulation complete — all modules exercised");
    log_print_all(&log);

    /* ======================================================
     * 9. STATS SUMMARY
     * ====================================================== */
    section("9. Final Client Stats");
    printf("  %-20s %12s %12s\n", "Client", "Bytes Sent", "Bytes Dropped");
    printf("  %s\n", "--------------------------------------------");
    for (int i = 0; i < 6; i++) {
        printf("  %-20s %12llu %12llu\n",
               all_clients[i]->name,
               (unsigned long long)all_clients[i]->bytes_sent,
               (unsigned long long)all_clients[i]->bytes_dropped);
    }

    /* ======================================================
     * CLEANUP
     * ====================================================== */
    bst_free(route_root);
    ct_free(&ct);
    log_free(&log);

    printf("\nSimulation finished. All memory freed.\n\n");
}
