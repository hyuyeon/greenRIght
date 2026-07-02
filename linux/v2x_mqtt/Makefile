CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -std=gnu11 -Icommon $(shell pkg-config --cflags libcjson 2>/dev/null)
LDLIBS  = -lmosquitto -lpthread -lm $(shell pkg-config --libs libcjson 2>/dev/null || echo -lcjson)

BIN_DIR = bin
COMMON_SRCS = common/vehicle_codec.c

.PHONY: all clean run run-broker

all: $(BIN_DIR)/vehicle_app $(BIN_DIR)/broker_monitor

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BIN_DIR)/vehicle_app: vehicle/vehicle.c $(COMMON_SRCS) | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BIN_DIR)/broker_monitor: broker/broker_monitor.c $(COMMON_SRCS) | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

clean:
	rm -rf $(BIN_DIR)

# 편의 타겟 (로컬 mosquitto 브로커가 127.0.0.1:1883 에서 떠 있다고 가정)
run: $(BIN_DIR)/vehicle_app
	./$(BIN_DIR)/vehicle_app 127.0.0.1 1883

run-broker: $(BIN_DIR)/broker_monitor
	./$(BIN_DIR)/broker_monitor 127.0.0.1 1883 0
