/* tests/test_all.c
 * Lightweight unit tests — no external framework, pure C.
 * Run:  make test
 *
 * Each test function returns 0 on pass, nonzero on fail.
 * A summary is printed at the end.
 */
#include "../include/mininet.h"
#include <assert.h>

static int passed = 0;
static int failed = 0;

#define CHECK(cond, msg) \
    do { \
        if (cond) { printf("  [PASS] %s\n", msg); passed++; } \
        else      { printf("  [FAIL] %s  (line %d)\n", msg, __LINE__); failed++; } \
    } while(0)

/* ---- Ring Buffer ---- */
static void test_ring_buffer(void) {
    printf("\n-- RingBuffer --\n");
    RingBuffer rb;
    rb_init(&rb);
    CHECK(rb_is_empty(&rb), "empty after init");
    CHECK(!rb_is_full(&rb), "not full after init");

    Packet p = {0};
    p.seq = 42;
    CHECK(rb_push(&rb, &p) == 0, "push succeeds");
    CHECK(!rb_is_empty(&rb), "not empty after push");

    /* Fill to capacity */
    for (int i = 1; i < RING_BUF_CAP; i++) {
        p.seq = (uint32_t)i;
        rb_push(&rb, &p);
    }
    CHECK(rb_is_full(&rb), "full after RING_BUF_CAP pushes");
    CHECK(rb_push(&rb, &p) == -1, "push on full returns -1");

    Packet out;
    CHECK(rb_pop(&rb, &out) == 0, "pop succeeds");
    CHECK(out.seq == 42, "FIFO order correct (seq==42 first)");

    /* Drain */
    while (!rb_is_empty(&rb)) rb_pop(&rb, &out);
    CHECK(rb_is_empty(&rb), "empty after drain");
    CHECK(rb_pop(&rb, &out) == -1, "pop on empty returns -1");
}

/* ---- Packet Queue ---- */
static void test_packet_queue(void) {
    printf("\n-- PacketQueue --\n");
    PacketQueue q;
    pq_init(&q);
    CHECK(pq_is_empty(&q), "empty after init");

    Packet p = {0};
    p.seq = 100;
    pq_enqueue(&q, &p);
    p.seq = 101;
    pq_enqueue(&q, &p);
    p.seq = 102;
    pq_enqueue(&q, &p);

    CHECK(q.size == 3, "size == 3 after 3 enqueues");

    Packet out;
    pq_dequeue(&q, &out);
    CHECK(out.seq == 100, "FIFO: first out is seq=100");
    pq_dequeue(&q, &out);
    CHECK(out.seq == 101, "FIFO: second out is seq=101");

    pq_free(&q);
    CHECK(pq_is_empty(&q), "empty after pq_free");
}

/* ---- Retry Stack ---- */
static void test_retry_stack(void) {
    printf("\n-- RetryStack --\n");
    RetryStack s;
    rs_init(&s);
    CHECK(rs_is_empty(&s), "empty after init");

    Packet p = {0};
    p.seq = 200; rs_push(&s, &p, 20);
    p.seq = 201; rs_push(&s, &p, 40);
    p.seq = 202; rs_push(&s, &p, 80);

    Packet out; uint32_t bo;
    rs_pop(&s, &out, &bo);
    CHECK(out.seq == 202, "LIFO: top is seq=202");
    CHECK(bo == 80,       "backoff correct (80ms)");
    rs_pop(&s, &out, &bo);
    CHECK(out.seq == 201, "LIFO: next is seq=201");

    rs_free(&s);
    CHECK(rs_is_empty(&s), "empty after rs_free");
}

/* ---- Client Table (Hash) ---- */
static void test_client_table(void) {
    printf("\n-- ClientTable (Hash) --\n");
    ClientTable ct;
    ct_init(&ct);

    uint8_t mac1[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
    uint8_t mac2[6] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};

    Client *c1 = client_new(mac1, "10.0.0.1", "DevA", -50, 5, BAND_5G, 36);
    Client *c2 = client_new(mac2, "10.0.0.2", "DevB", -60, 3, BAND_2G,  6);

    CHECK(ct_insert(&ct, c1) == 0, "insert c1 ok");
    CHECK(ct_insert(&ct, c2) == 0, "insert c2 ok");
    CHECK(ct_insert(&ct, c1) == -1, "duplicate insert returns -1");
    CHECK(ct.count == 2, "count == 2");

    Client *found = ct_find(&ct, mac1);
    CHECK(found != NULL, "find mac1 returns non-null");
    CHECK(strcmp(found->name, "DevA") == 0, "found correct name");

    CHECK(ct_find(&ct, mac2) != NULL, "find mac2 ok");
    CHECK(ct_remove(&ct, mac2) == 0, "remove mac2 ok");
    CHECK(ct_find(&ct, mac2) == NULL, "find mac2 after remove = NULL");
    CHECK(ct.count == 1, "count == 1 after remove");

    ct_free(&ct);
    CHECK(ct.count == 0, "count == 0 after ct_free");
}

/* ---- Routing BST ---- */
static void test_routing_bst(void) {
    printf("\n-- Routing BST --\n");
    RouteNode *root = NULL;
    uint8_t mac_a[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
    uint8_t mac_b[6] = {0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
    uint8_t mac_c[6] = {0xFF,0xFE,0xFD,0xFC,0xFB,0xFA};

    root = bst_insert(root, "192.168.1.10", mac_a);
    root = bst_insert(root, "192.168.1.5",  mac_b);
    root = bst_insert(root, "192.168.1.20", mac_c);

    RouteNode *r = bst_search(root, "192.168.1.10");
    CHECK(r != NULL, "search 192.168.1.10 found");
    CHECK(memcmp(r->mac, mac_a, 6) == 0, "correct MAC returned");

    CHECK(bst_search(root, "10.0.0.1") == NULL, "search missing IP = NULL");

    root = bst_delete(root, "192.168.1.10");
    CHECK(bst_search(root, "192.168.1.10") == NULL, "deleted node not found");
    CHECK(bst_search(root, "192.168.1.5")  != NULL, "sibling still found");

    bst_free(root);
    CHECK(1, "bst_free completed without crash");
}

/* ---- Scheduler (Heap) ---- */
static void test_scheduler(void) {
    printf("\n-- Scheduler (Heap) --\n");
    Scheduler s;
    sched_init(&s);

    uint8_t m1[6]={0x01}, m2[6]={0x02}, m3[6]={0x03};
    /* signal: -40 prio:5, -70 prio:9, -55 prio:3 */
    Client *a = client_new(m1, "1.1.1.1", "Alpha", -40, 5, BAND_5G, 36);
    Client *b = client_new(m2, "1.1.1.2", "Beta",  -70, 9, BAND_5G, 36);
    Client *c = client_new(m3, "1.1.1.3", "Gamma", -55, 3, BAND_5G, 36);

    sched_insert(&s, a);
    sched_insert(&s, b);
    sched_insert(&s, c);
    CHECK(s.size == 3, "heap size == 3");

    /* Alpha: key = 40 - 25 = 15
     * Beta:  key = 70 - 45 = 25
     * Gamma: key = 55 - 15 = 40
     * Min key = Alpha (best) */
    Client *best = sched_pop_best(&s);
    CHECK(strcmp(best->name, "Alpha") == 0, "best is Alpha (strongest signal+prio)");

    /* Clean up (clients allocated with calloc, not in hash table) */
    free(a); free(b); free(c);
    CHECK(1, "scheduler cleanup ok");
}

/* ---- AP Graph ---- */
static void test_ap_graph(void) {
    printf("\n-- AP Graph (BFS/DFS) --\n");
    APGraph g;
    graph_init(&g);

    int a = graph_add_ap(&g, "AP-A", BAND_5G, 36, 20);
    int b = graph_add_ap(&g, "AP-B", BAND_5G, 40, 18);
    int c = graph_add_ap(&g, "AP-C", BAND_2G,  6, 15);
    graph_add_edge(&g, a, b, 30);
    graph_add_edge(&g, b, c, 20);

    CHECK(g.ap_count == 3, "3 APs registered");
    CHECK(g.aps[a].neighbor_count == 1, "AP-A has 1 neighbor");
    CHECK(g.aps[b].neighbor_count == 2, "AP-B has 2 neighbors");

    printf("BFS output:\n");
    graph_bfs(&g, a);

    int visited[MAX_APS] = {0};
    printf("DFS output:\n");
    graph_dfs(&g, a, visited);
    CHECK(visited[a] && visited[b] && visited[c], "DFS visited all 3 APs");
}

/* ---- Event Log ---- */
static void test_event_log(void) {
    printf("\n-- EventLog (Dynamic Array) --\n");
    EventLog el;
    log_init(&el);
    CHECK(el.size == 0, "size==0 after init");
    CHECK(el.capacity == LOG_INIT_CAP, "capacity==LOG_INIT_CAP after init");

    for (int i = 0; i < LOG_INIT_CAP + 5; i++) {
        log_append(&el, "test entry %d", i);
    }
    CHECK(el.size == LOG_INIT_CAP + 5, "size correct after growth");
    CHECK(el.capacity >= LOG_INIT_CAP + 5, "capacity grew on overflow");

    log_free(&el);
    CHECK(el.entries == NULL, "entries NULL after free");
}

/* ---- MAC utils ---- */
static void test_utils(void) {
    printf("\n-- Utils (MAC / strings) --\n");
    uint8_t mac[6] = {0xDE,0xAD,0xBE,0xEF,0x12,0x34};
    char buf[18];
    mac_to_str(mac, buf);
    CHECK(strcmp(buf, "DE:AD:BE:EF:12:34") == 0, "mac_to_str correct");

    uint8_t mac2[6];
    CHECK(str_to_mac("DE:AD:BE:EF:12:34", mac2) == 0, "str_to_mac returns 0");
    CHECK(memcmp(mac, mac2, 6) == 0, "round-trip MAC equal");
    CHECK(str_to_mac("invalid", mac2) == -1, "str_to_mac invalid returns -1");
}

/* ---- main ---- */
int main(void) {
    printf("\n");
    printf("=======================================================\n");
    printf("  MiniNet — Unit Test Suite\n");
    printf("=======================================================\n");

    test_ring_buffer();
    test_packet_queue();
    test_retry_stack();
    test_client_table();
    test_routing_bst();
    test_scheduler();
    test_ap_graph();
    test_event_log();
    test_utils();

    printf("\n=======================================================\n");
    printf("  Results: %d passed, %d failed\n", passed, failed);
    printf("=======================================================\n\n");
    return failed == 0 ? 0 : 1;
}
