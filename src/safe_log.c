/* safe_log.c
 * Thread-safe event log: mutex-protected wrapper around EventLog.
 *
 * Pattern: every writer acquires the mutex, appends, releases.
 * Readers (print) also lock so they see a consistent snapshot.
 *
 * Concept exercised: mutex for shared mutable state.
 */
#include "mininet.h"
#include <stdarg.h>

void slog_init(SafeLog *sl) {
    log_init(&sl->log);
    pthread_mutex_init(&sl->mutex, NULL);
}

void slog_append(SafeLog *sl, const char *fmt, ...) {
    /* Build the formatted string first (outside the lock — cheap) */
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    /* Critical section: append to shared log */
    pthread_mutex_lock(&sl->mutex);
    log_append(&sl->log, "%s", buf);
    pthread_mutex_unlock(&sl->mutex);
}

void slog_print(SafeLog *sl) {
    pthread_mutex_lock(&sl->mutex);
    log_print_all(&sl->log);
    pthread_mutex_unlock(&sl->mutex);
}

void slog_free(SafeLog *sl) {
    pthread_mutex_lock(&sl->mutex);
    log_free(&sl->log);
    pthread_mutex_unlock(&sl->mutex);
    pthread_mutex_destroy(&sl->mutex);
}
