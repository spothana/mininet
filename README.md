# MiniNet — WiFi Network Simulator in C

A complete C project that exercises every fundamental data structure,
concurrency primitive, and lockless multi-core technique through the
lens of a real WiFi driver / embedded networking system.

---

## What this project covers

| Category | Concepts |
|---|---|
| Data structures | array, char, int, string, stack, queue, linked list, ring buffer, BST, min-heap, graph, hash table, dynamic array |
| Concurrency (Part 2) | `pthread_mutex`, `sem_t`, `pthread_cond`, `atomic_int`, full TX + RX paths |
| Lockless / multi-core (Part 3) | SPSC ring, MPSC queue, RCU table, CPU affinity, false-sharing elimination |

---

## Data Structure Map

| Module           | File                 | Data Structure                         |
|------------------|----------------------|----------------------------------------|
| Packet           | `include/mininet.h`  | `struct`, `uint8_t[]`, `char[]`, `int` |
| Ring buffer      | `src/ring_buffer.c`  | Circular array (fixed-cap)             |
| Packet queue     | `src/packet_queue.c` | Singly-linked list (FIFO)              |
| Retry stack      | `src/retry_stack.c`  | Singly-linked list (LIFO)              |
| Client table     | `src/client_table.c` | Hash table + linked list chaining      |
| Routing table    | `src/routing_bst.c`  | Binary Search Tree (IP → MAC)          |
| Scheduler        | `src/scheduler.c`    | Min-heap (priority queue)              |
| AP graph         | `src/ap_graph.c`     | Adjacency-list graph + BFS + DFS       |
| Event log        | `src/event_log.c`    | Dynamically resizable array            |

---

## Concurrency Map (Part 2)

| Module           | File                  | Primitive(s)                                      |
|------------------|-----------------------|---------------------------------------------------|
| Safe log         | `src/safe_log.c`      | `pthread_mutex` — shared log writer               |
| Shared queue     | `src/shared_queue.c`  | `sem_t` (slots + items) + `pthread_mutex`         |
| AP worker        | `src/threaded_sim.c`  | `pthread_cond work_ready` — sleeps when heap empty|
| Generator        | `src/threaded_sim.c`  | `atomic_uint` seq counter, signals `work_ready`   |
| Relay            | `src/threaded_sim.c`  | `pthread_cond relay_cond` — sleeps when queue empty|
| Receiver         | `src/threaded_sim.c`  | Per-client `pthread_cond rx_ready` (NAPI hook)    |
| Client (TX)      | `src/client_table.c`  | `pthread_mutex tx_mutex` guards `tx_queue`        |
| Client (RX)      | `src/client_table.c`  | `pthread_mutex rx_mutex` + `rx_ready` condvar     |
| Route table      | `src/routing_bst.c`   | `pthread_mutex` wrapping BST (rt_* API)           |

### Condition variable rules enforced throughout

```c
/* Always a while, never if — guards spurious wakeups */
while (heap_empty && !stop)
    pthread_cond_wait(&work_ready, &sched_mutex);

/* Signal while still holding the mutex — no lost-signal window */
sched_insert(...);
pthread_cond_signal(&work_ready);   /* still inside lock */
pthread_mutex_unlock(&sched_mutex);

/* Shutdown uses broadcast — wakes ALL waiting threads */
pthread_cond_broadcast(&work_ready);
```

---

## Lockless / Multi-core Map (Part 3)

| Module          | File                   | Technique                                  |
|-----------------|------------------------|--------------------------------------------|
| SPSC ring       | `src/spsc_ring.c`      | Lock-free circular buffer (release/acquire)|
| MPSC queue      | `src/mpsc_queue.c`     | Vyukov intrusive list (atomic_exchange)    |
| RCU table       | `src/rcu_table.c`      | Copy-then-swap, single acquire-load reader |
| CPU affinity    | `src/cpu_affinity.c`   | `pthread_setaffinity_np`, AlignedCounter   |
| Lockless sim    | `src/lockless_sim.c`   | Zero mutex on packet hot path              |

### Hot path synchronisation cost comparison

| Path | Part 2 (mutex) | Part 3 (lockless) |
|---|---|---|
| Generator → AP worker | lock `tx_mutex` + lock `sched_mutex` + `cond_signal` | `atomic_exchange` on MPSC tail |
| AP worker → relay | lock `sq.mutex` + `sem_wait` + `sem_post` + `cond_signal` | `atomic_store` on SPSC tail |
| Relay → client | lock `rx_mutex` + `rb_push` + `cond_signal` | `atomic_store` on SPSC tail |
| Client table lookup | lock `ct.mutex` | single `atomic_load_acquire` (RCU) |
| Per-core stats | `atomic_fetch_add` (shared cache line) | `AlignedCounter` (64-byte padded) |

### SPSC memory ordering rationale

```
Producer                         Consumer
─────────────────────────────    ─────────────────────────────
1. load head  (acquire)          1. load tail  (acquire)
2. slots[tail] = pkt  (plain)       ↑ sees producer's release store
3. store tail (RELEASE)  ──────→    guarantees slot write visible
                                 2. out = slots[head]  (plain)
                                 3. store head (RELEASE)
```

The `release` store on `tail` and the `acquire` load on `tail` form
a happens-before edge. No mutex needed because producer owns `tail`
(writes it) and consumer owns `head` (writes it) — they never race
on the same variable.

---

## Build & Run

```bash
# Build everything (main sim + test binary)
make

# Run all three simulation parts
make run
# or directly:
./mininet

# Run unit tests (53 tests, no external dependencies)
make test
# or:
./test_mininet

# Clean all build artefacts
make clean
```

Requires: `gcc`, `make`, Linux (for `pthread_setaffinity_np` and
`CPU_SET`). CPU pinning degrades gracefully on macOS — a warning is
printed but all algorithms remain correct.

---

## Project Layout

```
mininet/
├── include/
│   └── mininet.h               ← all types, constants, prototypes
│                                 (Part 1 structs + Part 2 sync types
│                                  + Part 3 lockless structs)
├── src/
│   ├── main.c                  ← entry point: runs Parts 1, 2, 3
│   ├── utils.c                 ← MAC helpers, timestamp, sleep_us
│   │
│   ├── ── Part 1: data structures ──────────────────────────────
│   ├── ring_buffer.c           ← circular array (DMA TX ring)
│   ├── packet_queue.c          ← FIFO linked list (TX queue)
│   ├── retry_stack.c           ← LIFO linked list (CSMA/CA)
│   ├── client_table.c          ← hash table MAC→Client
│   ├── routing_bst.c           ← BST IP→MAC + thread-safe rt_* API
│   ├── scheduler.c             ← min-heap QoS scheduler
│   ├── ap_graph.c              ← adjacency-list graph, BFS, DFS
│   ├── event_log.c             ← dynamic array event log
│   ├── simulation.c            ← Part 1 single-threaded walkthrough
│   │
│   ├── ── Part 2: mutex · semaphore · condvar ──────────────────
│   ├── safe_log.c              ← mutex-protected EventLog
│   ├── shared_queue.c          ← sem_slots/items bounded buffer
│   ├── threaded_sim.c          ← AP workers, generator, relay,
│   │                             receiver — full duplex TX + RX
│   │
│   └── ── Part 3: lockless multi-core ─────────────────────────
│       ├── spsc_ring.c         ← SPSC lock-free ring (release/acquire)
│       ├── mpsc_queue.c        ← MPSC Vyukov queue (atomic_exchange)
│       ├── rcu_table.c         ← RCU hash table (copy-then-swap)
│       ├── cpu_affinity.c      ← core pinning + AlignedCounter
│       └── lockless_sim.c      ← zero-mutex hot path simulation
│
├── tests/
│   └── test_all.c              ← 53 unit tests (no external deps)
├── Makefile
└── README.md
```

---

## Simulation Parts

### Part 1 — Single-threaded data structure walkthrough
Exercises every data structure in sequence with printed output.
No threads. The full scenario models a real WiFi deployment:

1. **AP graph** — 4 APs (5 GHz/2.4 GHz/6 GHz), interference edges,
   BFS roaming path, DFS coverage mapping.
2. **Client table** — 6 devices keyed by MAC address in a hash table
   with FNV-1a hashing and linked list chaining.
3. **Routing BST** — IP-to-MAC ARP table: insert, search, delete,
   in-order traversal.
4. **Ring buffer** — CarPlay video frames in a circular DMA TX ring.
5. **Packet queue** — Android Auto audio in a per-client FIFO queue.
6. **Retry stack** — CSMA/CA backoff: failed frames pushed LIFO with
   exponential backoff timers.
7. **Scheduler** — Min-heap grants airtime by signal strength and
   priority (models 802.11e EDCA).
8. **Event log** — Timestamped entries in a dynamically resized array.

### Part 2 — Multi-threaded full-duplex simulation
Five concurrent threads with TX and RX paths:

```
TX uplink:
  [Generator] → tx_mutex/cond_signal → [AP worker]
              → sem_slots/relay_cond → [Relay]
              → BFS roam decision

RX downlink:
  [Relay] → rx_queue SharedQueue → [Receiver]
          → ct_find (hash lookup) → rb_push(client->rx_ring)
          → cond_signal(rx_ready)
```

### Part 3 — Multi-core lockless simulation
Same logical flow, zero mutexes on the packet path:

```
[Generator] ──MPSC atomic_exchange──► [AP worker, pinned core N]
                                              │
                                        SPSC relay_ring
                                         (sole producer)
                                              │
                                       [Relay thread]
                                        RCU lookup (acquire-load only)
                                              │
                                     SPSC per-client rx_ring
                                      (sole producer per ring)
                                              │
                                       [Client consumer]
```

---

## Real-World Connections

| Sim component       | Real kernel/driver equivalent                    |
|---------------------|--------------------------------------------------|
| Ring buffer         | `mac80211` DMA TX/RX descriptor rings            |
| Packet queue        | Per-station `sk_buff` queues in `cfg80211`       |
| Retry stack         | 802.11 CSMA/CA exponential backoff               |
| Min-heap scheduler  | EDCA access categories (AC_VI for CarPlay video) |
| AP graph + BFS      | BSS roaming, MCC/SCC concurrency decisions       |
| Routing BST         | ARP cache / forwarding table                     |
| Hash table          | Station management table in driver               |
| `pthread_cond`      | Linux `wait_event` / `wake_up` in softirq path  |
| SPSC ring           | `mac80211` per-station lockless TX queues        |
| MPSC queue          | `io_uring` submission ring (Vyukov MPSC)         |
| RCU table           | `rcu_read_lock` / station table in `cfg80211`    |
| CPU affinity        | `ksoftirqd` / NAPI thread pinning                |
| `AlignedCounter`    | `per_cpu` counters in kernel network stack       |
