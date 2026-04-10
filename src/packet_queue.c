/* packet_queue.c
 * Singly-linked FIFO queue of Packets.
 * Models the per-station TX queue in mac80211's software queuing layer.
 *
 * Data structures exercised: linked list (singly), dynamic heap allocation
 */
#include "mininet.h"

void pq_init(PacketQueue *q) {
    q->front = NULL;
    q->rear  = NULL;
    q->size  = 0;
}

void pq_enqueue(PacketQueue *q, const Packet *pkt) {
    QNode *node = malloc(sizeof(QNode));
    if (!node) { fprintf(stderr, "pq_enqueue: OOM\n"); return; }
    node->pkt  = *pkt;
    node->next = NULL;
    if (q->rear == NULL) {
        q->front = q->rear = node;
    } else {
        q->rear->next = node;
        q->rear = node;
    }
    q->size++;
}

/* Dequeue into *out; returns 0 on success, -1 if empty */
int pq_dequeue(PacketQueue *q, Packet *out) {
    if (q->front == NULL) return -1;
    QNode *tmp = q->front;
    *out = tmp->pkt;
    q->front = tmp->next;
    if (q->front == NULL) q->rear = NULL;
    free(tmp);
    q->size--;
    return 0;
}

int pq_is_empty(const PacketQueue *q) {
    return q->front == NULL;
}

void pq_free(PacketQueue *q) {
    QNode *cur = q->front;
    while (cur) {
        QNode *nxt = cur->next;
        free(cur);
        cur = nxt;
    }
    q->front = q->rear = NULL;
    q->size = 0;
}

void pq_print(const PacketQueue *q) {
    printf("  PacketQueue [size=%d]\n", q->size);
    QNode *cur = q->front;
    int i = 0;
    while (cur) {
        char src[18], dst[18];
        mac_to_str(cur->pkt.src_mac, src);
        mac_to_str(cur->pkt.dst_mac, dst);
        printf("    [%d] seq=%-5u  %s -> %s  len=%u\n",
               i++, cur->pkt.seq, src, dst, cur->pkt.len);
        cur = cur->next;
    }
}
