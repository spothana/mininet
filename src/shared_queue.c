/* shared_queue.c
 * Bounded producer/consumer queue using POSIX semaphores + mutex.
 *
 * ┌─────────────────────────────────────────────────────────┐
 * │  PRODUCER                        CONSUMER               │
 * │                                                         │
 * │  sem_wait(&sem_slots)  ←blocks   sem_wait(&sem_items)   │
 * │  pthread_mutex_lock()            pthread_mutex_lock()   │
 * │    write slot[tail]                read slot[head]      │
 * │    tail++; count++               head++; count--        │
 * │  pthread_mutex_unlock()          pthread_mutex_unlock() │
 * │  sem_post(&sem_items)            sem_post(&sem_slots)   │
 * └─────────────────────────────────────────────────────────┘
 *
 * sem_slots starts at MAX_SHARED_QUEUE (all slots free).
 * sem_items starts at 0 (nothing to consume).
 *
 * Why both a mutex AND semaphores?
 *   - sem_slots/sem_items handle the BLOCKING: they let threads
 *     sleep efficiently instead of spin-waiting.
 *   - The mutex handles EXCLUSION: if two producers both pass
 *     sem_wait(sem_slots) before either writes, without the mutex
 *     they'd both write to slot[tail] simultaneously → data race.
 *
 * Concepts: semaphore, mutex, producer-consumer, ring buffer.
 */
#include "mininet.h"

void sq_init(SharedQueue *sq) {
    sq->head = sq->tail = sq->count = 0;
    pthread_mutex_init(&sq->mutex, NULL);
    sem_init(&sq->sem_slots, 0, MAX_SHARED_QUEUE); /* all slots free */
    sem_init(&sq->sem_items, 0, 0);                /* nothing ready  */
}

/* Blocking produce: waits if queue is full */
void sq_produce(SharedQueue *sq, const Packet *pkt) {
    sem_wait(&sq->sem_slots);           /* block until a slot is free */
    pthread_mutex_lock(&sq->mutex);
    sq->slots[sq->tail] = *pkt;
    sq->tail = (sq->tail + 1) % MAX_SHARED_QUEUE;
    sq->count++;
    pthread_mutex_unlock(&sq->mutex);
    sem_post(&sq->sem_items);           /* signal: one more item ready */
}

/* Non-blocking produce: returns 0 on success, -1 if full */
int sq_try_produce(SharedQueue *sq, const Packet *pkt) {
    if (sem_trywait(&sq->sem_slots) != 0) return -1;
    pthread_mutex_lock(&sq->mutex);
    sq->slots[sq->tail] = *pkt;
    sq->tail = (sq->tail + 1) % MAX_SHARED_QUEUE;
    sq->count++;
    pthread_mutex_unlock(&sq->mutex);
    sem_post(&sq->sem_items);
    return 0;
}

/* Blocking consume: waits if queue is empty */
void sq_consume(SharedQueue *sq, Packet *out) {
    sem_wait(&sq->sem_items);           /* block until an item is ready */
    pthread_mutex_lock(&sq->mutex);
    *out = sq->slots[sq->head];
    sq->head = (sq->head + 1) % MAX_SHARED_QUEUE;
    sq->count--;
    pthread_mutex_unlock(&sq->mutex);
    sem_post(&sq->sem_slots);           /* signal: one more slot free  */
}

/* Non-blocking consume: returns 0 on success, -1 if empty */
int sq_try_consume(SharedQueue *sq, Packet *out) {
    if (sem_trywait(&sq->sem_items) != 0) return -1;
    pthread_mutex_lock(&sq->mutex);
    *out = sq->slots[sq->head];
    sq->head = (sq->head + 1) % MAX_SHARED_QUEUE;
    sq->count--;
    pthread_mutex_unlock(&sq->mutex);
    sem_post(&sq->sem_slots);
    return 0;
}

void sq_destroy(SharedQueue *sq) {
    pthread_mutex_destroy(&sq->mutex);
    sem_destroy(&sq->sem_slots);
    sem_destroy(&sq->sem_items);
}
