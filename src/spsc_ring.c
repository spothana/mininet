/* spsc_ring.c
 * Lock-free Single Producer Single Consumer ring buffer.
 *
 * CORRECTNESS ARGUMENT
 * ────────────────────
 * Two threads share this structure: one producer (P) and one consumer (C).
 * They each own one index:
 *
 *   P owns tail: P writes tail (release), C reads tail (acquire)
 *   C owns head: C writes head (release), P reads head (acquire)
 *
 * The acquire/release pairing ensures that when C sees tail advance,
 * it also sees the slot write that preceded it. Symmetrically when P
 * sees head advance, it sees the slot read has completed.
 *
 * No other synchronisation is needed because:
 *   - P never reads or writes head beyond a single load.
 *   - C never reads or writes tail beyond a single load.
 *   - They access different slot indices (tail != head when non-empty).
 *
 * MEMORY ORDER CHOICES
 * ────────────────────
 *   atomic_store tail   → memory_order_release
 *     Ensures slot write (pkt copy) is visible BEFORE tail advance.
 *
 *   atomic_load  tail   → memory_order_acquire
 *     Ensures C sees slot write that happened BEFORE producer's release.
 *
 *   atomic_store head   → memory_order_release
 *     Ensures slot read is complete BEFORE head advances.
 *     (Important: producer must not overwrite a slot C is still reading.)
 *
 *   atomic_load  head   → memory_order_acquire
 *     Ensures P sees the completed read before it reuses the slot.
 *
 * On x86 these map to plain stores/loads (TSO makes release/acquire
 * free). On ARM they emit stlr/ldar (store-release/load-acquire).
 *
 * WHY POWER-OF-TWO CAPACITY
 * ──────────────────────────
 * (idx & SPSC_MASK) is a single bitwise-AND, one cycle.
 * (idx % SPSC_CAP)  is a division — ~20-40 cycles on most CPUs.
 * For a ring that is hit on every packet, this matters.
 */
#include "mininet.h"

void spsc_init(SpscRing *r) {
    atomic_store_explicit(&r->head, 0, memory_order_relaxed);
    atomic_store_explicit(&r->tail, 0, memory_order_relaxed);
}

int spsc_empty(const SpscRing *r) {
    uint_fast32_t h = atomic_load_explicit(&r->head, memory_order_acquire);
    uint_fast32_t t = atomic_load_explicit(&r->tail, memory_order_acquire);
    return h == t;
}

/*
 * spsc_push — called by the PRODUCER thread only.
 * Returns 0 on success, -1 if ring is full.
 *
 * Critical ordering:
 *   1. Load head (acquire) — see consumer's latest slot-free signal.
 *   2. Check capacity.
 *   3. Write slot (plain — no barrier needed yet).
 *   4. Store tail (release) — publish slot to consumer.
 */
int spsc_push(SpscRing *r, const Packet *pkt) {
    uint_fast32_t t    = atomic_load_explicit(&r->tail, memory_order_relaxed);
    uint_fast32_t next = (t + 1) & SPSC_MASK;
    uint_fast32_t h    = atomic_load_explicit(&r->head, memory_order_acquire);

    if (next == h) return -1;   /* full */

    r->slots[t & SPSC_MASK] = *pkt;   /* write slot before advancing tail */

    atomic_store_explicit(&r->tail, next, memory_order_release);
    return 0;
}

/*
 * spsc_pop — called by the CONSUMER thread only.
 * Returns 0 on success, -1 if ring is empty.
 *
 * Critical ordering:
 *   1. Load tail (acquire) — see producer's slot-written signal.
 *   2. Check empty.
 *   3. Read slot (plain).
 *   4. Store head (release) — signal producer slot is free.
 */
int spsc_pop(SpscRing *r, Packet *out) {
    uint_fast32_t h = atomic_load_explicit(&r->head, memory_order_relaxed);
    uint_fast32_t t = atomic_load_explicit(&r->tail, memory_order_acquire);

    if (h == t) return -1;   /* empty */

    *out = r->slots[h & SPSC_MASK];   /* read slot before advancing head */

    atomic_store_explicit(&r->head, (h + 1) & SPSC_MASK, memory_order_release);
    return 0;
}
