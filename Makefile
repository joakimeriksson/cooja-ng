CC = cc
CFLAGS = -O3 -Wall -Wextra -Wno-unused-parameter -std=c11 -I include -march=native -flto
LDFLAGS = -lm -flto

# Auto-detect GNU Lightning
LIGHTNING_CFLAGS := $(shell pkg-config --cflags lightning 2>/dev/null)
LIGHTNING_LIBS := $(shell pkg-config --libs lightning 2>/dev/null)

SRC_DIR = src
TEST_DIR = test
BUILD_DIR = build

SOURCES = $(SRC_DIR)/msp430_cpu.c \
          $(SRC_DIR)/msp430_config.c \
          $(SRC_DIR)/msp430_elf.c \
          $(SRC_DIR)/msp430_usart.c \
          $(SRC_DIR)/msp430_timer.c \
          $(SRC_DIR)/msp430_clock.c \
          $(SRC_DIR)/msp430_gpio.c \
          $(SRC_DIR)/msp430_decode.c \
          $(SRC_DIR)/msp430_platform.c \
          $(SRC_DIR)/cc2420.c

ifneq ($(LIGHTNING_LIBS),)
  CFLAGS += $(LIGHTNING_CFLAGS) -DHAVE_LIGHTNING
  LDFLAGS += $(LIGHTNING_LIBS)
  SOURCES += $(SRC_DIR)/msp430_jit.c
endif

OBJECTS = $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SOURCES))

TEST_SOURCES = $(TEST_DIR)/test_main.c \
               $(TEST_DIR)/test_correctness.c \
               $(TEST_DIR)/test_benchmark.c \
               $(TEST_DIR)/test_firmware.c \
               $(TEST_DIR)/test_multinode.c

TEST_OBJECTS = $(patsubst $(TEST_DIR)/%.c, $(BUILD_DIR)/test_%.o, $(TEST_SOURCES))

.PHONY: all clean test bench test-firmware

all: $(BUILD_DIR)/test_runner

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/test_%.o: $(TEST_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/test_runner: $(OBJECTS) $(TEST_OBJECTS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

test: $(BUILD_DIR)/test_runner
	./$(BUILD_DIR)/test_runner correctness -v

bench: $(BUILD_DIR)/test_runner
	./$(BUILD_DIR)/test_runner bench

test-firmware: $(BUILD_DIR)/test_runner
	./$(BUILD_DIR)/test_runner firmware

clean:
	rm -rf $(BUILD_DIR)

# Debug build
debug: CFLAGS = -O0 -g -Wall -Wextra -Wno-unused-parameter -std=c11 -I include -DDEBUG
debug: LDFLAGS = -lm
debug: clean $(BUILD_DIR)/test_runner

# Profile-guided optimization (Apple Clang)
PGO_CFLAGS = -O3 -std=c11 -I include -march=native
PGO_LDFLAGS = -lm
ifneq ($(LIGHTNING_LIBS),)
  PGO_CFLAGS += $(LIGHTNING_CFLAGS) -DHAVE_LIGHTNING
  PGO_LDFLAGS += $(LIGHTNING_LIBS)
endif

pgo:
	@echo "=== PGO Step 1: Instrumented build ==="
	rm -rf $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)
	$(CC) $(PGO_CFLAGS) -fprofile-instr-generate \
		$(SOURCES) $(TEST_SOURCES) -o $(BUILD_DIR)/test_runner $(PGO_LDFLAGS)
	@echo "=== PGO Step 2: Collecting profile data ==="
	LLVM_PROFILE_FILE="$(BUILD_DIR)/default.profraw" ./$(BUILD_DIR)/test_runner bench
	LLVM_PROFILE_FILE="$(BUILD_DIR)/default.profraw" ./$(BUILD_DIR)/test_runner correctness
	xcrun llvm-profdata merge -output=$(BUILD_DIR)/default.profdata $(BUILD_DIR)/default.profraw
	@echo "=== PGO Step 3: Optimized build ==="
	$(CC) $(CFLAGS) -fprofile-instr-use=$(BUILD_DIR)/default.profdata \
		$(SOURCES) $(TEST_SOURCES) -o $(BUILD_DIR)/test_runner $(LDFLAGS)
	@echo "=== PGO build complete ==="
