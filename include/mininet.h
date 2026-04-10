#ifndef MININET_H
#define MININET_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* =========================================================
 *  MiniNet — WiFi Network Simulator
 *  Covers: arrays, int, char, strings, stack, queue,
 *          linked list, ring buffer, BST, heap, graph
 * ========================================================= */

/* ---------- tuneable constants ---------- */
#define MAX_CLIENTS        32
#define MAX_APS            8
#define MAX_PACKET_DATA    256
#define RING_BUF_CAP       16      /* per-client TX ring buffer capacity */
#define MAX_RETRY_STACK    8       /* retry stack depth per client */
#define HASH_BUCKETS       64
#define LOG_INIT_CAP       64      /* dynamic event log initial capacity */
#define MAX_AP_NEIGHBORS   8       /* max edges in AP interference graph */

/* ---------- 802.11 band / channel ---------- */
typedef enum { BAND_2G = 0, BAND_5G, BAND_6G } Band;

/* ---------- Packet (the fundamental unit) ---------- */
typedef struct {
    uint32_t  seq;                    /* sequence number           */
    uint8_t   src_mac[6];             /* source MAC (6 bytes)      */
    uint8_t   dst_mac[6];             /* destination MAC           */
    uint16_t  len;                    /* payload length            */
    uint8_t   data[MAX_PACKET_DATA];  /* payload bytes             */
    uint8_t   retry_count;            /* how many times resent     */
    uint64_t  timestamp_ms;           /* when created              */
} Packet;

/* ---------- Ring Buffer (circular TX/RX queue) ---------- */
typedef struct {
    Packet    slots[RING_BUF_CAP];
    int       head;        /* read  index */
    int       tail;        /* write index */
    int       count;
} RingBuffer;

/* ---------- Queue node (per-client TX FIFO) ---------- */
typedef struct QNode {
    Packet       pkt;
    struct QNode *next;
} QNode;

typedef struct {
    QNode *front;
    QNode *rear;
    int    size;
} PacketQueue;

/* ---------- Stack node (retry backoff) ---------- */
typedef struct SNode {
    Packet       pkt;
    uint32_t     backoff_ticks;
    struct SNode *next;
} SNode;

typedef struct {
    SNode *top;
    int    size;
} RetryStack;

/* ---------- Client (hash-table entry, linked list chain) ---------- */
typedef struct Client {
    uint8_t      mac[6];             /* key: MAC address           */
    char         ip[16];             /* IPv4 string "A.B.C.D"      */
    char         name[32];           /* human label                */
    int          signal_dbm;         /* RSSI in dBm (-30 .. -90)   */
    int          priority;           /* scheduling priority 0..9   */
    Band         band;
    int          channel;
    RingBuffer   tx_ring;            /* outgoing ring buffer        */
    PacketQueue  tx_queue;           /* TX FIFO queue               */
    RetryStack   retry_stack;        /* failed-packet retry stack   */
    uint64_t     bytes_sent;
    uint64_t     bytes_dropped;
    struct Client *next;             /* hash-chain next pointer     */
} Client;

/* ---------- Hash table (MAC -> Client*) ---------- */
typedef struct {
    Client *buckets[HASH_BUCKETS];
    int     count;
} ClientTable;

/* ---------- Routing BST (IP -> MAC) ---------- */
typedef struct RouteNode {
    char            ip[16];
    uint8_t         mac[6];
    struct RouteNode *left;
    struct RouteNode *right;
} RouteNode;

/* ---------- Min-Heap scheduler (priority queue) ---------- */
typedef struct {
    Client  *clients[MAX_CLIENTS];
    int      size;
} Scheduler;

/* ---------- AP interference graph ---------- */
typedef struct {
    int  neighbor_id;
    int  interference_pct;   /* 0-100 */
} APEdge;

typedef struct {
    int     id;
    char    ssid[33];
    Band    band;
    int     channel;
    int     tx_power_dbm;
    APEdge  neighbors[MAX_AP_NEIGHBORS];
    int     neighbor_count;
} AccessPoint;

typedef struct {
    AccessPoint aps[MAX_APS];
    int         ap_count;
} APGraph;

/* ---------- Dynamic event log ---------- */
typedef struct {
    char     msg[128];
    uint64_t ts_ms;
} LogEntry;

typedef struct {
    LogEntry *entries;
    int       size;
    int       capacity;
} EventLog;

/* =========================================================
 *  Function prototypes — one per module
 * ========================================================= */

/* utils */
uint64_t now_ms(void);
void     mac_to_str(const uint8_t mac[6], char out[18]);
int      str_to_mac(const char *str, uint8_t mac[6]);
void     print_mac(const uint8_t mac[6]);
int      ip_cmp(const char *a, const char *b);

/* ring_buffer.c */
void   rb_init(RingBuffer *rb);
int    rb_push(RingBuffer *rb, const Packet *pkt);
int    rb_pop(RingBuffer *rb, Packet *out);
int    rb_is_full(const RingBuffer *rb);
int    rb_is_empty(const RingBuffer *rb);
void   rb_print(const RingBuffer *rb);

/* packet_queue.c */
void   pq_init(PacketQueue *q);
void   pq_enqueue(PacketQueue *q, const Packet *pkt);
int    pq_dequeue(PacketQueue *q, Packet *out);
int    pq_is_empty(const PacketQueue *q);
void   pq_free(PacketQueue *q);
void   pq_print(const PacketQueue *q);

/* retry_stack.c */
void   rs_init(RetryStack *s);
int    rs_push(RetryStack *s, const Packet *pkt, uint32_t backoff);
int    rs_pop(RetryStack *s, Packet *out, uint32_t *backoff);
int    rs_is_empty(const RetryStack *s);
void   rs_free(RetryStack *s);

/* client_table.c */
uint32_t   mac_hash(const uint8_t mac[6]);
Client    *client_new(const uint8_t mac[6], const char *ip,
                      const char *name, int signal, int priority,
                      Band band, int channel);
void       ct_init(ClientTable *ct);
int        ct_insert(ClientTable *ct, Client *c);
Client    *ct_find(ClientTable *ct, const uint8_t mac[6]);
int        ct_remove(ClientTable *ct, const uint8_t mac[6]);
void       ct_print(const ClientTable *ct);
void       ct_free(ClientTable *ct);

/* routing_bst.c */
RouteNode *bst_insert(RouteNode *root, const char *ip, const uint8_t mac[6]);
RouteNode *bst_search(RouteNode *root, const char *ip);
RouteNode *bst_delete(RouteNode *root, const char *ip);
void       bst_inorder(const RouteNode *root);
void       bst_free(RouteNode *root);

/* scheduler.c (min-heap on signal_dbm inverted = best signal first) */
void   sched_init(Scheduler *s);
int    sched_insert(Scheduler *s, Client *c);
Client*sched_pop_best(Scheduler *s);
void   sched_print(const Scheduler *s);

/* ap_graph.c */
void   graph_init(APGraph *g);
int    graph_add_ap(APGraph *g, const char *ssid, Band band,
                    int channel, int tx_power_dbm);
void   graph_add_edge(APGraph *g, int ap_a, int ap_b, int interference_pct);
void   graph_bfs(const APGraph *g, int start_id);
void   graph_dfs(const APGraph *g, int start_id, int visited[]);
void   graph_print(const APGraph *g);

/* event_log.c */
void   log_init(EventLog *el);
void   log_append(EventLog *el, const char *fmt, ...);
void   log_print_all(const EventLog *el);
void   log_free(EventLog *el);

/* simulation.c */
void   sim_run(void);

#endif /* MININET_H */
