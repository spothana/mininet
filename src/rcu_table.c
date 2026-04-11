/* rcu_table.c
 * RCU-style hash table: readers pay zero synchronisation cost.
 *
 * RCU OVERVIEW
 * ────────────
 * RCU (Read-Copy-Update) is the standard technique in the Linux kernel
 * for data that is read millions of times per second but written rarely.
 * The rules:
 *
 *   READERS:
 *     1. Atomically load the current bucket array pointer.
 *     2. Walk the hash chain.  No lock, no barrier beyond the load.
 *     3. Return the result.  If a writer swaps the array mid-read,
 *        the reader still holds a valid pointer to the OLD array —
 *        safe until the reader is done because the writer only frees
 *        the old array AFTER a "grace period" where all readers have
 *        passed through a quiescent point.
 *
 *   WRITERS:
 *     1. Lock write_lock (serialises concurrent writers only).
 *     2. Allocate a new RcuBucketArray.
 *     3. Deep-copy all chains from old → new.
 *     4. Apply the update (insert/delete) on the new copy.
 *     5. atomic_store(&buckets, new_ptr, release)
 *        Readers that start AFTER this point see the new array.
 *        Readers that started BEFORE still use the old pointer.
 *     6. Grace period: usleep(1000) — in a real system this would
 *        be synchronize_rcu() which waits for all CPUs to pass a
 *        context switch (quiescent state). In our simulator a short
 *        sleep is sufficient because our reader threads are not
 *        pinned inside a read-side critical section for longer.
 *     7. Free the old array and its chains.
 *
 * WHY NOT JUST A RWLOCK?
 * ──────────────────────
 * pthread_rwlock allows many concurrent readers, but every reader
 * still acquires the lock — a cache-line write on the lock word,
 * potentially bouncing between cores on every lookup.
 * RCU readers touch ZERO shared writable state. The only shared
 * read is the atomic pointer load, which on x86 is a plain MOV
 * (TSO guarantees it is sequentially consistent with prior stores).
 *
 * SIMPLIFICATION vs PRODUCTION RCU
 * ──────────────────────────────────
 * Production RCU (Linux kernel, liburcu) uses epoch counters or
 * per-CPU quiescent-state tracking for the grace period. Our
 * usleep(1) approximation is safe for a simulator where writer
 * calls are infrequent (client join/leave), not the packet hot path.
 */
#include "mininet.h"

/* ---- internal helpers ---- */

static uint32_t rcu_hash(const uint8_t mac[6]) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
    return h % RCU_BUCKETS;
}

static RcuBucketArray *alloc_buckets(void) {
    RcuBucketArray *b = calloc(1, sizeof(RcuBucketArray));
    return b;
}

/* Deep-copy all chains from src into dst (dst must be freshly zeroed) */
static void copy_buckets(RcuBucketArray *dst, const RcuBucketArray *src) {
    for (int i = 0; i < RCU_BUCKETS; i++) {
        RcuEntry *e = src->chains[i];
        RcuEntry **tail = &dst->chains[i];
        while (e) {
            RcuEntry *n = malloc(sizeof(RcuEntry));
            *n = *e;
            n->next = NULL;
            *tail = n;
            tail = &n->next;
            e = e->next;
        }
    }
}

static void free_buckets(RcuBucketArray *b) {
    for (int i = 0; i < RCU_BUCKETS; i++) {
        RcuEntry *e = b->chains[i];
        while (e) { RcuEntry *nxt = e->next; free(e); e = nxt; }
    }
    free(b);
}

/* ---- public API ---- */

void rcu_table_init(RcuTable *t) {
    RcuBucketArray *b = alloc_buckets();
    atomic_store_explicit(&t->buckets, b, memory_order_release);
    pthread_mutex_init(&t->write_lock, NULL);
    t->count = 0;
}

/*
 * rcu_table_lookup — READER hot path.
 *
 * The ONLY synchronisation is the single acquire load of the pointer.
 * No mutex, no reference count, no cache-line writes.
 * On x86 this compiles to: MOV + hash + chain walk.
 */
RcuEntry *rcu_table_lookup(RcuTable *t, const uint8_t mac[6]) {
    /* Acquire: see all writes that happened before the writer's release store */
    RcuBucketArray *b = atomic_load_explicit(&t->buckets, memory_order_acquire);
    uint32_t idx = rcu_hash(mac);
    RcuEntry *e  = b->chains[idx];
    while (e) {
        if (memcmp(e->mac, mac, 6) == 0) return e;
        e = e->next;
    }
    return NULL;
}

/*
 * rcu_table_insert — WRITER path.
 *
 * Copy-then-swap: readers always see a complete, consistent array.
 */
void rcu_table_insert(RcuTable *t, const uint8_t mac[6],
                      const char *name, const char *ip,
                      int ap_id, int signal_dbm) {
    pthread_mutex_lock(&t->write_lock);

    RcuBucketArray *old = atomic_load_explicit(&t->buckets, memory_order_relaxed);
    RcuBucketArray *new = alloc_buckets();
    copy_buckets(new, old);

    /* Insert new entry into new copy */
    uint32_t  idx = rcu_hash(mac);
    RcuEntry *e   = malloc(sizeof(RcuEntry));
    memcpy(e->mac, mac, 6);
    strncpy(e->name, name, sizeof(e->name)-1);
    strncpy(e->ip,   ip,   sizeof(e->ip)  -1);
    e->ap_id      = ap_id;
    e->signal_dbm = signal_dbm;
    e->next       = new->chains[idx];
    new->chains[idx] = e;

    /* Publish new array — readers that load after this see the insert */
    atomic_store_explicit(&t->buckets, new, memory_order_release);
    t->count++;

    pthread_mutex_unlock(&t->write_lock);

    /*
     * Grace period: wait for any readers that loaded old to finish.
     * In our simulation GEN_TICK_US (60ms) >> any lookup time,
     * so 1ms is a safe quiescent approximation.
     */
    sleep_us(1000);
    free_buckets(old);
}

void rcu_table_free(RcuTable *t) {
    pthread_mutex_lock(&t->write_lock);
    RcuBucketArray *b = atomic_load_explicit(&t->buckets, memory_order_relaxed);
    atomic_store_explicit(&t->buckets, NULL, memory_order_release);
    pthread_mutex_unlock(&t->write_lock);
    if (b) free_buckets(b);
    pthread_mutex_destroy(&t->write_lock);
}
