#ifndef MININET_H
#define MININET_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdatomic.h>

/* =========================================================
 *  MiniNet — WiFi Network Simulator
 *  Covers: arrays, int, char, strings, stack, queue,
 *          linked list, ring buffer, BST, heap, graph,
 *          mutex, semaphore, threads
 * ========================================================= */

/* ---------- tuneable constants ---------- */
#define MAX_CLIENTS         32
#define MAX_APS             8
#define MAX_PACKET_DATA     256
#define RING_BUF_CAP        16
#define MAX_RETRY_STACK     8
#define HASH_BUCKETS        64
#define LOG_INIT_CAP        64
#define MAX_AP_NEIGHBORS    8

/* ---------- threading constants ---------- */
#define SIM_DURATION_S      4       /* total simulation wall-clock seconds */
#define AP_TICK_US          120000  /* AP scheduler tick (120ms)           */
#define GEN_TICK_US         60000   /* packet generator tick (60ms)        */
#define RELAY_TICK_US       90000   /* relay monitor tick (90ms)           */
#define MAX_SHARED_QUEUE    128     /* inter-AP relay queue depth          */

/* ---------- 802.11 band / channel ---------- */
typedef enum { BAND_2G = 0, BAND_5G, BAND_6G } Band;

/* ---------- Packet ---------- */
typedef struct {
    uint32_t seq;
    uint8_t  src_mac[6];
    uint8_t  dst_mac[6];
    uint16_t len;
    uint8_t  data[MAX_PACKET_DATA];
    uint8_t  retry_count;
    uint64_t timestamp_ms;
    int      ap_id;
} Packet;

/* ---------- Ring Buffer ---------- */
typedef struct {
    Packet slots[RING_BUF_CAP];
    int    head, tail, count;
} RingBuffer;

/* ---------- Packet Queue (FIFO linked list) ---------- */
typedef struct QNode { Packet pkt; struct QNode *next; } QNode;
typedef struct { QNode *front, *rear; int size; } PacketQueue;

/* ---------- Retry Stack (LIFO linked list) ---------- */
typedef struct SNode { Packet pkt; uint32_t backoff_ticks; struct SNode *next; } SNode;
typedef struct { SNode *top; int size; } RetryStack;

/* ---------- Client ---------- */
typedef struct Client {
    uint8_t         mac[6];
    char            ip[16];
    char            name[32];
    int             signal_dbm;
    int             priority;
    Band            band;
    int             channel;
    int             ap_id;
    RingBuffer      tx_ring;
    PacketQueue     tx_queue;
    RetryStack      retry_stack;
    pthread_mutex_t tx_mutex;     /* guards tx_ring + tx_queue */
    uint64_t        bytes_sent;
    uint64_t        bytes_dropped;
    struct Client  *next;
} Client;

/* ---------- Hash table (MAC -> Client*) ---------- */
typedef struct {
    Client         *buckets[HASH_BUCKETS];
    int             count;
    pthread_mutex_t mutex;
} ClientTable;

/* ---------- Routing BST (IP -> MAC) ---------- */
typedef struct RouteNode {
    char              ip[16];
    uint8_t           mac[6];
    struct RouteNode *left, *right;
} RouteNode;

typedef struct {
    RouteNode      *root;
    pthread_mutex_t mutex;
} RouteTable;

/* ---------- Min-Heap Scheduler ---------- */
typedef struct { Client *clients[MAX_CLIENTS]; int size; } Scheduler;

/* ---------- AP Graph ---------- */
typedef struct { int neighbor_id; int interference_pct; } APEdge;

typedef struct {
    int    id;
    char   ssid[33];
    Band   band;
    int    channel;
    int    tx_power_dbm;
    APEdge neighbors[MAX_AP_NEIGHBORS];
    int    neighbor_count;
} AccessPoint;

typedef struct { AccessPoint aps[MAX_APS]; int ap_count; } APGraph;

/* ---------- Dynamic event log ---------- */
typedef struct { char msg[128]; uint64_t ts_ms; } LogEntry;
typedef struct { LogEntry *entries; int size, capacity; } EventLog;

/* =========================================================
 *  CONCURRENCY LAYER
 * ========================================================= */

/*
 * SharedQueue  —  bounded producer/consumer buffer between threads.
 *
 * MUTEX + SEMAPHORE pattern (classic bounded buffer):
 *
 *   sem_slots  : counts FREE slots.  Starts at MAX_SHARED_QUEUE.
 *                Producer calls sem_wait(&sem_slots) before writing —
 *                blocks when the queue is full.
 *
 *   sem_items  : counts FILLED slots. Starts at 0.
 *                Consumer calls sem_wait(&sem_items) before reading —
 *                blocks when the queue is empty.
 *
 *   mutex      : short critical section that protects head/tail/count
 *                so only one thread touches the indices at a time.
 *
 * Why both semaphores AND a mutex?
 *   Semaphores handle blocking/signalling across threads.
 *   The mutex prevents a race if two producers (or two consumers)
 *   run concurrently — sem_wait only serialises count, not the
 *   read/write of the slot itself.
 */
typedef struct {
    Packet          slots[MAX_SHARED_QUEUE];
    int             head, tail, count;
    pthread_mutex_t mutex;
    sem_t           sem_slots;      /* producer waits here when full  */
    sem_t           sem_items;      /* consumer waits here when empty */
    /* condvar used by relay thread — AP workers signal after producing */
    pthread_mutex_t relay_mutex;
    pthread_cond_t  relay_cond;
} SharedQueue;

/*
 * SafeLog  —  mutex-protected EventLog.
 * Any thread appending acquires the mutex first, guaranteeing
 * that log entries are never interleaved or corrupted.
 */
typedef struct {
    EventLog        log;
    pthread_mutex_t mutex;
} SafeLog;

/*
 * APWorker  —  one pthread per Access Point.
 * See work_ready condvar comment in the struct below.
 */
/*
 * work_ready — condition variable paired with sched_mutex.
 *
 * AP worker thread calls:
 *   pthread_mutex_lock(&sched_mutex)
 *   while (scheduler.size == 0 && !stop)
 *       pthread_cond_wait(&work_ready, &sched_mutex)  ← sleeps, releases mutex
 *   ... do work ...
 *   pthread_mutex_unlock(&sched_mutex)
 *
 * Generator thread calls, after inserting a client into the heap:
 *   pthread_cond_signal(&work_ready)   ← wakes the AP worker
 *
 * pthread_cond_wait atomically releases sched_mutex and sleeps,
 * so there is no window where a signal can be missed between the
 * "heap is empty" check and the sleep.
 *
 * On shutdown: ap_worker_stop() calls pthread_cond_broadcast()
 * so the worker wakes, sees stop==1, and exits cleanly.
 */
typedef struct {
    int              ap_id;
    AccessPoint     *ap;
    Client          *clients[MAX_CLIENTS];
    int              client_count;
    pthread_mutex_t  sched_mutex;
    pthread_cond_t   work_ready;     /* signalled when heap becomes non-empty */
    Scheduler        scheduler;
    SharedQueue     *outbound;
    SafeLog         *slog;
    atomic_int       stop;
    pthread_t        tid;
    atomic_uint_fast64_t packets_tx;
    atomic_uint_fast64_t packets_dropped;
} APWorker;

/*
 * GeneratorCtx  —  one packet-generator pthread.
 *
 * Simulates devices sending data: every GEN_TICK_US it picks a
 * random client, builds a Packet, and enqueues it into that
 * client's tx_queue (under tx_mutex).
 *
 * atomic_uint seq_counter gives each packet a unique sequence
 * number without needing a mutex — atomic increment is sufficient.
 */
typedef struct {
    Client      **clients;
    int           client_count;
    APWorker     *workers;
    int           worker_count;
    SafeLog      *slog;
    atomic_int    stop;
    atomic_uint   seq_counter;   /* lock-free monotonic counter */
    pthread_t     tid;
} GeneratorCtx;

/*
 * RelayCtx  —  one relay/monitor pthread.
 *
 * Drains all AP outbound queues, logs packet flow, and uses BFS
 * on the AP graph to decide whether to "roam" a packet to a
 * neighbouring AP (simulating 802.11r fast BSS transition).
 */
/*
 * relay_cond — condition variable that wakes the relay thread.
 *
 * AP worker signals relay_cond after successfully producing a packet
 * to the outbound SharedQueue.  Relay thread waits on it when all
 * AP outbound queues are empty, avoiding a busy RELAY_TICK_US poll.
 *
 * relay_mutex is a dedicated lightweight mutex just for the condvar —
 * it does NOT protect the SharedQueue itself (that uses its own mutex
 * + semaphores); it only serialises the "is there anything to relay?"
 * check so that pthread_cond_wait can be used correctly.
 */
typedef struct {
    APWorker        *workers;
    int              worker_count;
    APGraph         *graph;
    SafeLog         *slog;
    atomic_int       stop;
    pthread_t        tid;
    /* relay_mutex and relay_cond live on SharedQueue (outbound),
     * so all AP workers can signal the same relay thread via one condvar */
    atomic_uint_fast64_t packets_relayed;
} RelayCtx;

/* =========================================================
 *  Function prototypes
 * ========================================================= */

/* utils.c */
uint64_t now_ms(void);
void     mac_to_str(const uint8_t mac[6], char out[18]);
int      str_to_mac(const char *str, uint8_t mac[6]);
void     print_mac(const uint8_t mac[6]);
int      ip_cmp(const char *a, const char *b);
void     sleep_us(long us);

/* ring_buffer.c */
void rb_init(RingBuffer *rb);
int  rb_push(RingBuffer *rb, const Packet *pkt);
int  rb_pop(RingBuffer *rb, Packet *out);
int  rb_is_full(const RingBuffer *rb);
int  rb_is_empty(const RingBuffer *rb);
void rb_print(const RingBuffer *rb);

/* packet_queue.c */
void pq_init(PacketQueue *q);
void pq_enqueue(PacketQueue *q, const Packet *pkt);
int  pq_dequeue(PacketQueue *q, Packet *out);
int  pq_is_empty(const PacketQueue *q);
void pq_free(PacketQueue *q);
void pq_print(const PacketQueue *q);

/* retry_stack.c */
void rs_init(RetryStack *s);
int  rs_push(RetryStack *s, const Packet *pkt, uint32_t backoff);
int  rs_pop(RetryStack *s, Packet *out, uint32_t *backoff);
int  rs_is_empty(const RetryStack *s);
void rs_free(RetryStack *s);

/* client_table.c */
uint32_t  mac_hash(const uint8_t mac[6]);
Client   *client_new(const uint8_t mac[6], const char *ip,
                     const char *name, int signal, int priority,
                     Band band, int channel, int ap_id);
void      ct_init(ClientTable *ct);
int       ct_insert(ClientTable *ct, Client *c);
Client   *ct_find(ClientTable *ct, const uint8_t mac[6]);
int       ct_remove(ClientTable *ct, const uint8_t mac[6]);
void      ct_print(const ClientTable *ct);
void      ct_free(ClientTable *ct);

/* routing_bst.c */
RouteNode *bst_insert(RouteNode *root, const char *ip, const uint8_t mac[6]);
RouteNode *bst_search(RouteNode *root, const char *ip);
RouteNode *bst_delete(RouteNode *root, const char *ip);
void       bst_inorder(const RouteNode *root);
void       bst_free(RouteNode *root);
void       rt_init(RouteTable *rt);
void       rt_insert(RouteTable *rt, const char *ip, const uint8_t mac[6]);
RouteNode *rt_search(RouteTable *rt, const char *ip);
void       rt_delete(RouteTable *rt, const char *ip);
void       rt_print(RouteTable *rt);
void       rt_free(RouteTable *rt);

/* scheduler.c */
void    sched_init(Scheduler *s);
int     sched_insert(Scheduler *s, Client *c);
Client *sched_pop_best(Scheduler *s);
void    sched_print(const Scheduler *s);

/* ap_graph.c */
void graph_init(APGraph *g);
int  graph_add_ap(APGraph *g, const char *ssid, Band band, int channel, int tx_power_dbm);
void graph_add_edge(APGraph *g, int ap_a, int ap_b, int interference_pct);
void graph_bfs(const APGraph *g, int start_id);
void graph_dfs(const APGraph *g, int start_id, int visited[]);
void graph_print(const APGraph *g);

/* event_log.c */
void log_init(EventLog *el);
void log_append(EventLog *el, const char *fmt, ...);
void log_print_all(const EventLog *el);
void log_free(EventLog *el);

/* safe_log.c */
void slog_init(SafeLog *sl);
void slog_append(SafeLog *sl, const char *fmt, ...);
void slog_print(SafeLog *sl);
void slog_free(SafeLog *sl);

/* shared_queue.c */
void sq_init(SharedQueue *sq);
void sq_produce(SharedQueue *sq, const Packet *pkt);
int  sq_try_produce(SharedQueue *sq, const Packet *pkt);
void sq_consume(SharedQueue *sq, Packet *out);
int  sq_try_consume(SharedQueue *sq, Packet *out);
void sq_destroy(SharedQueue *sq);

/* threaded_sim.c */
void ap_worker_init(APWorker *w, int ap_id, AccessPoint *ap,
                    SharedQueue *outbound, SafeLog *slog);
void ap_worker_add_client(APWorker *w, Client *c);
void ap_worker_start(APWorker *w);
void ap_worker_stop(APWorker *w);
void generator_start(GeneratorCtx *g);
void generator_stop(GeneratorCtx *g);
void relay_start(RelayCtx *r);
void relay_stop(RelayCtx *r);

/* simulation.c */
void sim_run(void);
void sim_run_threaded(void);

#endif /* MININET_H */
