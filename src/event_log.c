/* event_log.c
 * Dynamically-resizable array of timestamped log strings.
 * Doubles capacity when full (amortised O(1) append).
 *
 * Data structures exercised: dynamic array, realloc, char[], sprintf
 */
#include "mininet.h"
#include <stdarg.h>

void log_init(EventLog *el) {
    el->entries  = malloc(LOG_INIT_CAP * sizeof(LogEntry));
    el->size     = 0;
    el->capacity = LOG_INIT_CAP;
}

void log_append(EventLog *el, const char *fmt, ...) {
    if (el->size == el->capacity) {
        el->capacity *= 2;
        el->entries   = realloc(el->entries, el->capacity * sizeof(LogEntry));
        if (!el->entries) { fprintf(stderr, "log_append: OOM\n"); return; }
    }
    LogEntry *e = &el->entries[el->size++];
    e->ts_ms = now_ms();
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(e->msg, sizeof(e->msg), fmt, ap);
    va_end(ap);
}

void log_print_all(const EventLog *el) {
    printf("EventLog [%d entries]\n", el->size);
    for (int i = 0; i < el->size; i++) {
        printf("  [%4d] +%6llums  %s\n",
               i, (unsigned long long)el->entries[i].ts_ms, el->entries[i].msg);
    }
}

void log_free(EventLog *el) {
    free(el->entries);
    el->entries  = NULL;
    el->size     = el->capacity = 0;
}
