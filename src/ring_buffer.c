/* ring_buffer.c
 * Fixed-capacity circular buffer for packets.
 * Models the kernel-level DMA ring used in real WiFi drivers (mac80211).
 *
 * Data structures exercised: array, int indexing, struct
 */
#include "mininet.h"

void rb_init(RingBuffer *rb) {
    rb->head  = 0;
    rb->tail  = 0;
    rb->count = 0;
    memset(rb->slots, 0, sizeof(rb->slots));
}

int rb_is_full(const RingBuffer *rb) {
    return rb->count == RING_BUF_CAP;
}

int rb_is_empty(const RingBuffer *rb) {
    return rb->count == 0;
}

/* Push a packet into the ring; returns 0 on success, -1 if full */
int rb_push(RingBuffer *rb, const Packet *pkt) {
    if (rb_is_full(rb)) return -1;
    rb->slots[rb->tail] = *pkt;
    rb->tail = (rb->tail + 1) % RING_BUF_CAP;
    rb->count++;
    return 0;
}

/* Pop the oldest packet; returns 0 on success, -1 if empty */
int rb_pop(RingBuffer *rb, Packet *out) {
    if (rb_is_empty(rb)) return -1;
    *out = rb->slots[rb->head];
    rb->head = (rb->head + 1) % RING_BUF_CAP;
    rb->count--;
    return 0;
}

void rb_print(const RingBuffer *rb) {
    printf("  RingBuffer [count=%d/%d head=%d tail=%d]\n",
           rb->count, RING_BUF_CAP, rb->head, rb->tail);
    for (int i = 0; i < rb->count; i++) {
        int idx = (rb->head + i) % RING_BUF_CAP;
        const Packet *p = &rb->slots[idx];
        char src[18], dst[18];
        mac_to_str(p->src_mac, src);
        mac_to_str(p->dst_mac, dst);
        printf("    [%d] seq=%-5u  %s -> %s  len=%u  retry=%u\n",
               idx, p->seq, src, dst, p->len, p->retry_count);
    }
}
