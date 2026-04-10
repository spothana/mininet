/* main.c — MiniNet WiFi Network Simulator */
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
    printf("  Arrays · Strings · Stack · Queue · LinkedList · RingBuf\n");
    printf("  BST · Heap · Graph · HashTable · Mutex · Semaphore · Threads\n");

    /* ---- Part 1: single-threaded walkthrough of all data structures ---- */
    printf("\n\n");
    printf("══════════════════════════════════════════════════════\n");
    printf("  PART 1 — Single-threaded data structure walkthrough\n");
    printf("══════════════════════════════════════════════════════\n");
    sim_run();

    /* ---- Part 2: multi-threaded concurrent simulation ---- */
    printf("\n");
    printf("══════════════════════════════════════════════════════\n");
    printf("  PART 2 — Multi-threaded concurrent simulation\n");
    printf("══════════════════════════════════════════════════════\n");
    sim_run_threaded();

    return 0;
}
