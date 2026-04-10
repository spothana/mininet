/* client_table.c
 * Hash table: MAC address (6 bytes) -> Client*
 * Collision resolution: separate chaining (linked list per bucket).
 *
 * Data structures exercised: array, linked list, hash function, strings,
 *                             uint8_t[], char[]
 */
#include "mininet.h"

/* FNV-1a over 6 MAC bytes */
uint32_t mac_hash(const uint8_t mac[6]) {
    uint32_t h = 2166136261u;
    for (int i = 0; i < 6; i++) {
        h ^= mac[i];
        h *= 16777619u;
    }
    return h % HASH_BUCKETS;
}

/* Allocate and initialise a new Client */
Client *client_new(const uint8_t mac[6], const char *ip,
                   const char *name, int signal, int priority,
                   Band band, int channel, int ap_id) {
    Client *c = calloc(1, sizeof(Client));
    if (!c) return NULL;
    memcpy(c->mac, mac, 6);
    strncpy(c->ip,   ip,   sizeof(c->ip)   - 1);
    strncpy(c->name, name, sizeof(c->name) - 1);
    c->signal_dbm = signal;
    c->priority   = priority;
    c->band       = band;
    c->channel    = channel;
    c->ap_id      = ap_id;
    pthread_mutex_init(&c->tx_mutex, NULL);
    rb_init(&c->tx_ring);
    rb_init(&c->rx_ring);
    pthread_mutex_init(&c->rx_mutex, NULL);
    pthread_cond_init(&c->rx_ready, NULL);
    pq_init(&c->tx_queue);
    rs_init(&c->retry_stack);
    c->next = NULL;
    return c;
}

void ct_init(ClientTable *ct) {
    memset(ct->buckets, 0, sizeof(ct->buckets));
    ct->count = 0;
    pthread_mutex_init(&ct->mutex, NULL);
}

/* Insert; returns 0 ok, -1 duplicate or error */
int ct_insert(ClientTable *ct, Client *c) {
    uint32_t idx = mac_hash(c->mac);
    /* Check for duplicate */
    Client *cur = ct->buckets[idx];
    while (cur) {
        if (memcmp(cur->mac, c->mac, 6) == 0) return -1;
        cur = cur->next;
    }
    c->next = ct->buckets[idx];
    ct->buckets[idx] = c;
    ct->count++;
    return 0;
}

/* Find by MAC; returns Client* or NULL */
Client *ct_find(ClientTable *ct, const uint8_t mac[6]) {
    uint32_t idx = mac_hash(mac);
    Client *cur = ct->buckets[idx];
    while (cur) {
        if (memcmp(cur->mac, mac, 6) == 0) return cur;
        cur = cur->next;
    }
    return NULL;
}

/* Remove by MAC; returns 0 ok, -1 not found */
int ct_remove(ClientTable *ct, const uint8_t mac[6]) {
    uint32_t idx = mac_hash(mac);
    Client *cur  = ct->buckets[idx];
    Client *prev = NULL;
    while (cur) {
        if (memcmp(cur->mac, mac, 6) == 0) {
            if (prev) prev->next = cur->next;
            else      ct->buckets[idx] = cur->next;
            pq_free(&cur->tx_queue);
            rs_free(&cur->retry_stack);
            pthread_mutex_destroy(&cur->tx_mutex);
            pthread_cond_destroy(&cur->rx_ready);
            pthread_mutex_destroy(&cur->rx_mutex);
            free(cur);
            ct->count--;
            return 0;
        }
        prev = cur;
        cur  = cur->next;
    }
    return -1;
}

static const char *band_str(Band b) {
    switch (b) {
        case BAND_2G: return "2.4GHz";
        case BAND_5G: return "5GHz";
        case BAND_6G: return "6GHz";
        default:      return "?";
    }
}

void ct_print(const ClientTable *ct) {
    printf("ClientTable [%d clients]\n", ct->count);
    printf("  %-18s %-15s %-20s %6s %4s %-6s %3s\n",
           "MAC", "IP", "Name", "Signal", "Prio", "Band", "Ch");
    printf("  %s\n", "---------------------------------------------------------------------");
    for (int i = 0; i < HASH_BUCKETS; i++) {
        Client *cur = ct->buckets[i];
        while (cur) {
            char mac_s[18];
            mac_to_str(cur->mac, mac_s);
            printf("  %-18s %-15s %-20s %4ddBm %4d %-6s %3d\n",
                   mac_s, cur->ip, cur->name, cur->signal_dbm,
                   cur->priority, band_str(cur->band), cur->channel);
            cur = cur->next;
        }
    }
}

void ct_free(ClientTable *ct) {
    for (int i = 0; i < HASH_BUCKETS; i++) {
        Client *cur = ct->buckets[i];
        while (cur) {
            Client *nxt = cur->next;
            pq_free(&cur->tx_queue);
            rs_free(&cur->retry_stack);
            pthread_mutex_destroy(&cur->tx_mutex);
            pthread_cond_destroy(&cur->rx_ready);
            pthread_mutex_destroy(&cur->rx_mutex);
            free(cur);
            cur = nxt;
        }
        ct->buckets[i] = NULL;
    }
    ct->count = 0;
}
