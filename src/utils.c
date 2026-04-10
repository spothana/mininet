#include "mininet.h"
#include <stdarg.h>

/* millisecond timestamp */
uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL;
}

/* uint8_t[6] -> "AA:BB:CC:DD:EE:FF" */
void mac_to_str(const uint8_t mac[6], char out[18]) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* "AA:BB:CC:DD:EE:FF" -> uint8_t[6], returns 0 on success */
int str_to_mac(const char *str, uint8_t mac[6]) {
    unsigned int v[6];
    if (sscanf(str, "%02X:%02X:%02X:%02X:%02X:%02X",
               &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) != 6)
        return -1;
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)v[i];
    return 0;
}

void print_mac(const uint8_t mac[6]) {
    char buf[18];
    mac_to_str(mac, buf);
    printf("%s", buf);
}

/* strcmp-style compare for dotted-quad IPs */
int ip_cmp(const char *a, const char *b) {
    return strcmp(a, b);
}

void sleep_us(long us) {
    struct timespec ts = { .tv_sec = us / 1000000L, .tv_nsec = (us % 1000000L) * 1000L };
    nanosleep(&ts, NULL);
}
