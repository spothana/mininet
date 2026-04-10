/* retry_stack.c
 * LIFO stack for failed packets that need retransmission after backoff.
 * Models CSMA/CA exponential backoff retry in 802.11.
 *
 * Data structures exercised: linked list used as stack, dynamic allocation
 */
#include "mininet.h"

void rs_init(RetryStack *s) {
    s->top  = NULL;
    s->size = 0;
}

/* Push packet + backoff_ticks onto stack; returns 0 ok, -1 if at depth limit */
int rs_push(RetryStack *s, const Packet *pkt, uint32_t backoff) {
    if (s->size >= MAX_RETRY_STACK) return -1;
    SNode *node = malloc(sizeof(SNode));
    if (!node) return -1;
    node->pkt           = *pkt;
    node->backoff_ticks = backoff;
    node->next          = s->top;
    s->top              = node;
    s->size++;
    return 0;
}

/* Pop into *out; returns 0 ok, -1 if empty */
int rs_pop(RetryStack *s, Packet *out, uint32_t *backoff) {
    if (!s->top) return -1;
    SNode *tmp  = s->top;
    *out        = tmp->pkt;
    *backoff    = tmp->backoff_ticks;
    s->top      = tmp->next;
    free(tmp);
    s->size--;
    return 0;
}

int rs_is_empty(const RetryStack *s) {
    return s->top == NULL;
}

void rs_free(RetryStack *s) {
    while (s->top) {
        SNode *tmp = s->top;
        s->top = tmp->next;
        free(tmp);
    }
    s->size = 0;
}
