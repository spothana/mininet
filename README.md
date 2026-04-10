# MiniNet — WiFi Network Simulator in C

A complete C project that exercises every fundamental data structure
through the lens of a real WiFi driver / embedded networking system.

---

## Data Structure Map

| Module            | File                  | Data Structure(s)                      |
|-------------------|-----------------------|----------------------------------------|
| Packet            | `include/mininet.h`   | `struct`, `uint8_t[]`, `char[]`, `int` |
| Ring Buffer       | `src/ring_buffer.c`   | Circular array (fixed-cap)             |
| Packet Queue      | `src/packet_queue.c`  | Singly-linked list (FIFO)              |
| Retry Stack       | `src/retry_stack.c`   | Singly-linked list (LIFO / stack)      |
| Client Table      | `src/client_table.c`  | Hash table + linked list chaining      |
| Routing Table     | `src/routing_bst.c`   | Binary Search Tree (IP string keys)    |
| Scheduler         | `src/scheduler.c`     | Min-heap (priority queue)              |
| AP Graph          | `src/ap_graph.c`      | Adjacency-list graph + BFS + DFS       |
| Event Log         | `src/event_log.c`     | Dynamically resizable array            |

---

## Build & Run

```bash
# Build everything
make

# Run the full simulation
make run
# or
./mininet

# Run unit tests
make test
# or
./test_mininet

# Clean build artifacts
make clean
```

Requires: `gcc`, `make`, POSIX `clock_gettime` (Linux / macOS).

---

## Project Layout

```
mininet/
├── include/
│   └── mininet.h          ← all types, constants, prototypes
├── src/
│   ├── main.c             ← entry point + ASCII banner
│   ├── utils.c            ← MAC string helpers, timestamp
│   ├── ring_buffer.c      ← circular buffer (DMA ring)
│   ├── packet_queue.c     ← FIFO TX queue (linked list)
│   ├── retry_stack.c      ← CSMA/CA retry stack
│   ├── client_table.c     ← hash table (MAC → Client)
│   ├── routing_bst.c      ← BST routing table (IP → MAC)
│   ├── scheduler.c        ← min-heap QoS scheduler
│   ├── ap_graph.c         ← AP interference graph (BFS/DFS)
│   ├── event_log.c        ← dynamic array event logger
│   └── simulation.c       ← full end-to-end scenario
├── tests/
│   └── test_all.c         ← unit tests (no external deps)
├── Makefile
└── README.md
```

---

## Simulation Scenario

The simulation models a real WiFi deployment:

1. **AP Graph** — 4 access points (5GHz/2.4GHz/6GHz) with interference
   edges. BFS finds roaming paths; DFS maps coverage.

2. **Client Table** — 6 devices (iPhone CarPlay, Pixel Android Auto,
   iPad, Samsung phone, MacBook, IoT sensor) keyed by MAC address in a
   hash table with chaining.

3. **Routing BST** — IP-to-MAC ARP table stored in a BST; demonstrates
   insert, search, delete, in-order traversal.

4. **Ring Buffer** — CarPlay video frames injected into a circular DMA
   ring, then drained.

5. **Packet Queue** — Android Auto audio chunks in a per-client FIFO TX
   queue.

6. **Retry Stack** — Simulated packet loss pushes frames onto a LIFO
   retry stack with exponential backoff (CSMA/CA).

7. **Scheduler** — Min-heap grants airtime to clients in best-signal /
   highest-priority order (models 802.11e QoS / EDCA).

8. **Event Log** — All events timestamped in a dynamically growing array.

---

## Real-World Connections

| Sim Component     | Real Equivalent                                  |
|-------------------|--------------------------------------------------|
| Ring Buffer       | `mac80211` DMA TX/RX descriptor rings            |
| Packet Queue      | Per-station `sk_buff` queues in `cfg80211`       |
| Retry Stack       | 802.11 CSMA/CA exponential backoff               |
| Scheduler / Heap  | EDCA access categories (AC_VI for CarPlay video) |
| AP Graph / BFS    | BSS roaming decisions, MCC/SCC concurrency       |
| Routing BST       | ARP cache / forwarding table                     |
| Hash Table        | Station management table in driver               |
