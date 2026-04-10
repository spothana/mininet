/* scheduler.c
 * Min-heap priority scheduler: client with best signal (highest dBm) gets
 * airtime first.  We store -signal_dbm so the "smallest" number = best signal.
 *
 * Data structures exercised: heap (array-based), int arithmetic
 */
#include "mininet.h"

/* priority key: lower value = higher scheduling priority */
static int key(const Client *c) {
    /* Combine signal strength and app-level priority.
     * -signal_dbm: -(-30) = 30 (good), -(-90) = 90 (bad)
     * subtract priority (0-9) so higher priority lowers the key */
    return (-c->signal_dbm) - c->priority * 5;
}

static void swap(Scheduler *s, int i, int j) {
    Client *tmp = s->clients[i];
    s->clients[i] = s->clients[j];
    s->clients[j] = tmp;
}

static void sift_up(Scheduler *s, int i) {
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (key(s->clients[i]) < key(s->clients[parent])) {
            swap(s, i, parent);
            i = parent;
        } else break;
    }
}

static void sift_down(Scheduler *s, int i) {
    int n = s->size;
    while (1) {
        int left  = 2*i + 1;
        int right = 2*i + 2;
        int best  = i;
        if (left  < n && key(s->clients[left])  < key(s->clients[best])) best = left;
        if (right < n && key(s->clients[right]) < key(s->clients[best])) best = right;
        if (best == i) break;
        swap(s, i, best);
        i = best;
    }
}

void sched_init(Scheduler *s) {
    s->size = 0;
}

int sched_insert(Scheduler *s, Client *c) {
    if (s->size >= MAX_CLIENTS) return -1;
    s->clients[s->size++] = c;
    sift_up(s, s->size - 1);
    return 0;
}

/* Remove and return the highest-priority client */
Client *sched_pop_best(Scheduler *s) {
    if (s->size == 0) return NULL;
    Client *best = s->clients[0];
    s->clients[0] = s->clients[--s->size];
    if (s->size > 0) sift_down(s, 0);
    return best;
}

void sched_print(const Scheduler *s) {
    printf("Scheduler heap [%d clients]\n", s->size);
    printf("  %-20s %-15s %6s %4s  key\n", "Name", "IP", "Signal", "Prio");
    printf("  %s\n", "------------------------------------------------------");
    for (int i = 0; i < s->size; i++) {
        const Client *c = s->clients[i];
        printf("  [%d] %-20s %-15s %4ddBm %4d  %d\n",
               i, c->name, c->ip, c->signal_dbm, c->priority, key(c));
    }
}
