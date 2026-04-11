#define _GNU_SOURCE
/* cpu_affinity.c
 * CPU pinning and false-sharing elimination helpers.
 *
 * CPU AFFINITY
 * ────────────
 * pthread_setaffinity_np() binds a thread to a specific set of CPU
 * cores. Once pinned:
 *   - The thread's stack, TLS, and hot data structures stay in that
 *     core's L1/L2 cache between scheduling intervals.
 *   - The OS scheduler will not migrate the thread to another core
 *     (unless explicitly allowed by the cpu_set).
 *   - Lock-free data structures that are "producer-only" or
 *     "consumer-only" benefit most: the owning core never experiences
 *     a cache miss on its own index variables.
 *
 * In MiniNet: each LocklessAP thread is pinned to one core.
 * Its scheduler heap, local stats, and SPSC tail pointer all live
 * in that core's cache — zero cross-core coherence traffic on the
 * hot path.
 *
 * FALSE SHARING DETECTION (manual)
 * ─────────────────────────────────
 * Cachegrind / perf c2c / Intel VTune can detect false sharing.
 * The symptom is high "LLC misses" on stores to apparently
 * uncontended variables.
 *
 * Our mitigation: AlignedCounter pads each atomic counter to 64 bytes.
 * We verify the size at compile time with _Static_assert.
 */
#include "mininet.h"

#ifdef __linux__
#include <sched.h>

/*
 * pin_thread_to_core — bind the calling thread to a single CPU core.
 *
 * Returns 0 on success, -1 if the core index is out of range or
 * the kernel call fails (e.g. not enough cores, no permission).
 *
 * Fallback: if pinning fails we print a warning but continue — the
 * lockless algorithms remain correct, just potentially slower due to
 * migration.
 */
int pin_thread_to_core(int core_id) {
    int ncpus = (int)sysconf(_SC_NPROCESSORS_ONLN);
    if (core_id < 0 || core_id >= ncpus) {
        fprintf(stderr, "[AFFINITY] core %d unavailable (system has %d)\n",
                core_id, ncpus);
        return -1;
    }
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET((size_t)core_id, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
        perror("[AFFINITY] pthread_setaffinity_np");
        return -1;
    }
    return 0;
}

int num_online_cores(void) {
    return (int)sysconf(_SC_NPROCESSORS_ONLN);
}

#else
/* Non-Linux stub — affinity not available but code still compiles */
int pin_thread_to_core(int core_id) {
    (void)core_id;
    fprintf(stderr, "[AFFINITY] CPU pinning not supported on this OS\n");
    return -1;
}
int num_online_cores(void) { return 1; }
#endif

/*
 * Compile-time false-sharing guard.
 *
 * AlignedCounter must be exactly CACHE_LINE_SIZE bytes so that two
 * consecutive AlignedCounters in an array each occupy their own line.
 * If the struct is smaller (e.g. because a compiler packs it) or
 * larger (padding overflow), this assertion fires at compile time.
 */
_Static_assert(sizeof(AlignedCounter) == CACHE_LINE_SIZE,
    "AlignedCounter must be exactly one cache line — check CACHE_LINE_SIZE");

/*
 * aligned_counter_add / aligned_counter_load
 *
 * Thin wrappers so call sites read as domain language rather than
 * raw atomic intrinsics. Use relaxed ordering for per-core stats:
 * we only read them at shutdown (after a join fence), so no
 * ordering guarantee is needed on the increment itself.
 */
void aligned_counter_add(AlignedCounter *c, uint_fast64_t n) {
    atomic_fetch_add_explicit(&c->value, n, memory_order_relaxed);
}

uint_fast64_t aligned_counter_load(const AlignedCounter *c) {
    return atomic_load_explicit(&c->value, memory_order_relaxed);
}

void aligned_counter_init(AlignedCounter *c) {
    atomic_store_explicit(&c->value, 0, memory_order_relaxed);
}
