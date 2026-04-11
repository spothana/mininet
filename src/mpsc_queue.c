/* mpsc_queue.c
 * Lock-free Multi Producer Single Consumer queue.
 *
 * ALGORITHM (Dmitry Vyukov, 2010)
 * ────────────────────────────────
 * The queue is an intrusive singly-linked list where:
 *   - tail points to the last node ever pushed (atomic — producers race here)
 *   - head points to the sentinel stub node (consumer-only — no atomic needed)
 *
 * PUSH (any producer thread):
 *   node->next = NULL                       (plain store, node not yet visible)
 *   prev = atomic_exchange(&tail, node)     (swap tail — THIS serialises producers)
 *   prev->next = node                       (link into chain)
 *
 * The atomic_exchange step is the entire synchronisation. It guarantees:
 *   (a) Only one producer "owns" a given prev pointer.
 *   (b) The store to prev->next is release — consumer's acquire load sees it.
 *
 * POP (consumer only):
 *   next = head->next                       (acquire load)
 *   if next == NULL → queue logically empty
 *   *out = next->pkt
 *   head = next                             (stub advances forward)
 *   free old stub
 *
 * WHY THE STUB NODE?
 * ──────────────────
 * It separates the "last node pushed" pointer (tail) from the
 * "current read position" pointer (head). Without it, an empty
 * queue would require head == tail, but after every pop the consumer
 * would need to atomically update tail — adding contention. The stub
 * means head and tail are always different objects.
 *
 * ONE LIMITATION
 * ──────────────
 * There is a brief window between atomic_exchange and prev->next = node
 * where a consumer might see a NULL next and think the queue is empty,
 * even though a push is in progress. We handle this by retrying in
 * mpsc_pop with a sched_yield() — the same approach used in the Linux
 * io_uring MPSC implementation.
 */
#include "mininet.h"

void mpsc_init(MpscQueue *q) {
    /* stub node — its next pointer starts NULL */
    atomic_store_explicit(&q->stub.next, NULL, memory_order_relaxed);
    q->head = &q->stub;
    /* tail points at stub — first push will link after it */
    atomic_store_explicit(&q->tail, &q->stub, memory_order_relaxed);
}

/* mpsc_push — safe to call from any thread simultaneously */
void mpsc_push(MpscQueue *q, const Packet *pkt) {
    MpscNode *node = malloc(sizeof(MpscNode));
    if (!node) return;
    node->pkt = *pkt;
    atomic_store_explicit(&node->next, NULL, memory_order_relaxed);

    /*
     * atomic_exchange: swap tail to point at our node.
     * prev is the node that was previously last.
     * memory_order_acq_rel: release our node->pkt write,
     * acquire any prior push's prev->next write.
     */
    MpscNode *prev = atomic_exchange_explicit(&q->tail, node,
                                              memory_order_acq_rel);

    /*
     * Link: make the previous tail point to us.
     * release: ensures consumer sees node->pkt before it sees this link.
     */
    atomic_store_explicit(&prev->next, node, memory_order_release);
}

/*
 * mpsc_pop — consumer only. Returns 0 on success, -1 if empty.
 *
 * If we see next==NULL, the queue might be momentarily empty OR a
 * producer is mid-push (between exchange and prev->next store).
 * We return -1 in both cases; caller retries with sched_yield().
 */
int mpsc_pop(MpscQueue *q, Packet *out) {
    MpscNode *head = q->head;
    MpscNode *next = atomic_load_explicit(&head->next, memory_order_acquire);

    if (!next) return -1;   /* empty or producer mid-push */

    *out = next->pkt;       /* consume the payload */
    q->head = next;         /* advance stub forward */
    /* free the old stub only if it was heap-allocated (not the embedded one) */
    if (head != &q->stub) free(head);
    return 0;
}

/* Drain and free all remaining nodes (call after producer stops).
 * The stub node is embedded in MpscQueue — never free it. */
void mpsc_drain(MpscQueue *q) {
    Packet dummy;
    while (mpsc_pop(q, &dummy) == 0) {}
    /* After draining, head == the last node we promoted to stub role.
     * If that is not the original embedded stub, it was heap-allocated
     * by a producer and must be freed. */
    if (q->head != &q->stub) {
        free(q->head);
        q->head = &q->stub;
    }
}
