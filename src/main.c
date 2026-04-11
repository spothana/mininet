#include "mininet.h"

int main(void) {
    printf("\n");
    printf("  ███╗   ███╗██╗███╗   ██╗██╗███╗   ██╗███████╗████████╗\n");
    printf("  ████╗ ████║██║████╗  ██║██║████╗  ██║██╔════╝╚══██╔══╝\n");
    printf("  ██╔████╔██║██║██╔██╗ ██║██║██╔██╗ ██║█████╗     ██║   \n");
    printf("  ██║╚██╔╝██║██║██║╚██╗██║██║██║╚██╗██║██╔══╝     ██║   \n");
    printf("  ██║ ╚═╝ ██║██║██║ ╚████║██║██║ ╚████║███████╗   ██║   \n");
    printf("  ╚═╝     ╚═╝╚═╝╚═╝  ╚═══╝╚═╝╚═╝  ╚═══╝╚══════╝   ╚═╝  \n");
    printf("  WiFi Network Simulator  |  C Data Structures Practice\n");
    printf("  Part 1: data structures  |  Part 2: mutex+semaphore+cond\n");
    printf("  Part 3: SPSC·MPSC·RCU·CPU affinity (lockless)\n");

    printf("\n\n══════════════════════════════════════════════════════\n");
    printf("  PART 1 — Single-threaded data structure walkthrough\n");
    printf("══════════════════════════════════════════════════════\n");
    sim_run();

    printf("\n══════════════════════════════════════════════════════\n");
    printf("  PART 2 — Multi-threaded: mutex · semaphore · condvar\n");
    printf("══════════════════════════════════════════════════════\n");
    sim_run_threaded();

    printf("\n══════════════════════════════════════════════════════\n");
    printf("  PART 3 — Multi-core lockless: SPSC · MPSC · RCU\n");
    printf("══════════════════════════════════════════════════════\n");
    sim_run_lockless();

    return 0;
}
