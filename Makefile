# MiniNet Makefile
CC      = gcc
CFLAGS  = -Wall -Wextra -Wpedantic -std=c11 -Iinclude -g \
          -D_POSIX_C_SOURCE=200809L -pthread

SRCS    = src/utils.c \
          src/ring_buffer.c \
          src/packet_queue.c \
          src/retry_stack.c \
          src/client_table.c \
          src/routing_bst.c \
          src/scheduler.c \
          src/ap_graph.c \
          src/event_log.c \
          src/safe_log.c \
          src/shared_queue.c \
          src/simulation.c \
          src/threaded_sim.c

OBJS        = $(SRCS:.c=.o)
TARGET      = mininet
TEST_TARGET = test_mininet

.PHONY: all clean test run

all: $(TARGET) $(TEST_TARGET)

$(TARGET): $(OBJS) src/main.o
	$(CC) $(CFLAGS) -o $@ $^

$(TEST_TARGET): $(OBJS) tests/test_all.o
	$(CC) $(CFLAGS) -o $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

clean:
	rm -f $(OBJS) src/main.o tests/test_all.o $(TARGET) $(TEST_TARGET)
