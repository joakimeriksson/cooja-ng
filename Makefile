CC = cc
CFLAGS = -O3 -Wall -Wextra -Wno-unused-parameter -std=c11 -D_GNU_SOURCE -I include/common -I include/sim -I include/msp430 -I include/arm -I include/native -I include/ui -I src/motes -I lib -I lib/quickjs -march=native -flto -MMD -MP
LDFLAGS = -lm -lpthread -flto

# Auto-detect GNU Lightning
LIGHTNING_CFLAGS := $(shell pkg-config --cflags lightning 2>/dev/null)
LIGHTNING_LIBS := $(shell pkg-config --libs lightning 2>/dev/null)

COMMON_SRC_DIR = src/common
SIM_SRC_DIR = src/sim
SERVICES_SRC_DIR = src/services
MOTES_SRC_DIR = src/motes
MSP430_SRC_DIR = src/msp430
ARM_SRC_DIR = src/arm
NATIVE_SRC_DIR = src/native
UI_SRC_DIR = src/ui
LIB_SRC_DIR = lib
TEST_DIR = test
BUILD_DIR = build
COMMON_BUILD_DIR = build/common
SIM_BUILD_DIR = build/sim
SERVICES_BUILD_DIR = build/services
MOTES_BUILD_DIR = build/motes
MSP430_BUILD_DIR = build/msp430
ARM_BUILD_DIR = build/arm
NATIVE_BUILD_DIR = build/native
UI_BUILD_DIR = build/ui
LIB_BUILD_DIR = build/lib

SOURCES = $(MSP430_SRC_DIR)/msp430_cpu.c \
          $(MSP430_SRC_DIR)/msp430_config.c \
          $(MSP430_SRC_DIR)/msp430_elf.c \
          $(MSP430_SRC_DIR)/msp430_usart.c \
          $(MSP430_SRC_DIR)/msp430_timer.c \
          $(MSP430_SRC_DIR)/msp430_clock.c \
          $(MSP430_SRC_DIR)/msp430_gpio.c \
          $(MSP430_SRC_DIR)/msp430_decode.c \
          $(MSP430_SRC_DIR)/msp430_platform.c \
          $(MSP430_SRC_DIR)/cc2420.c

ARM_SOURCES = $(ARM_SRC_DIR)/arm_cpu.c \
              $(ARM_SRC_DIR)/arm_config.c \
              $(ARM_SRC_DIR)/arm_nvic.c \
              $(ARM_SRC_DIR)/arm_systick.c \
              $(ARM_SRC_DIR)/arm_elf.c \
              $(ARM_SRC_DIR)/arm_gdb.c \
              $(ARM_SRC_DIR)/arm_platform.c \
              $(ARM_SRC_DIR)/arm_vfp.c \
              $(ARM_SRC_DIR)/cc2538_soc.c \
              $(ARM_SRC_DIR)/nrf52840_soc.c \
              $(ARM_SRC_DIR)/nrf54l15_soc.c \
              $(ARM_SRC_DIR)/nrf_radio_common.c \
              $(ARM_SRC_DIR)/cc2538_uart.c \
              $(ARM_SRC_DIR)/cc2538_gpio.c \
              $(ARM_SRC_DIR)/cc2538_gptimer.c \
              $(ARM_SRC_DIR)/cc2538_sys_ctrl.c \
              $(ARM_SRC_DIR)/cc2538_ioc.c \
              $(ARM_SRC_DIR)/cc2538_rfcore.c \
              $(ARM_SRC_DIR)/cc2538_sleeptimer.c \
              $(ARM_SRC_DIR)/cc2538_ssi.c \
              $(ARM_SRC_DIR)/cc1200.c

COMMON_SOURCES = $(COMMON_SRC_DIR)/elf_loader.c \
                 $(COMMON_SRC_DIR)/radio_medium.c \
                 $(COMMON_SRC_DIR)/packet_analyzer.c \
                 $(COMMON_SRC_DIR)/timeline.c \
                 $(COMMON_SRC_DIR)/js_test_engine.c \
                 $(COMMON_SRC_DIR)/sim_event_queue.c \
                 $(COMMON_SRC_DIR)/gdb_stub.c \
                 $(COMMON_SRC_DIR)/pcap_writer.c \
                 $(COMMON_SRC_DIR)/mock_sim_host.c

# Simulation kernel — see docs/design/refactor-plan.md.
SIM_SOURCES = $(SIM_SRC_DIR)/sim_runtime.c \
              $(SIM_SRC_DIR)/sim_service.c \
              $(SIM_SRC_DIR)/sim_serial_bridge.c \
              $(SIM_SRC_DIR)/sim_external_command.c \
              $(SIM_SRC_DIR)/sim_radio_bus.c \
              $(SIM_SRC_DIR)/sim_board.c \
              $(SIM_SRC_DIR)/sim_registry.c \
              $(SIM_SRC_DIR)/sim_plugin.c \
              $(SIM_SRC_DIR)/sim_config.c

SERVICES_SOURCES = $(SERVICES_SRC_DIR)/timeline_service.c \
                   $(SERVICES_SRC_DIR)/pcap_service.c \
                   $(SERVICES_SRC_DIR)/progress_service.c \
                   $(SERVICES_SRC_DIR)/json_test_service.c \
                   $(SERVICES_SRC_DIR)/js_test_service.c \
                   $(SERVICES_SRC_DIR)/gdb_service.c \
                   $(SERVICES_SRC_DIR)/websocket_ui_service.c

# Per-kind mote modules (boot policy + adapters) + the mote-kind
# registry — Phase 4, §3.17.
MOTES_SOURCES = $(MOTES_SRC_DIR)/js_app_mote.c \
                $(MOTES_SRC_DIR)/native_cooja_mote.c \
                $(MOTES_SRC_DIR)/msp430_elf_mote.c \
                $(MOTES_SRC_DIR)/arm_elf_mote.c \
                $(MOTES_SRC_DIR)/mote_kinds.c

NATIVE_SOURCES = $(NATIVE_SRC_DIR)/native_node.c \
                 $(NATIVE_SRC_DIR)/native_radio.c \
                 $(NATIVE_SRC_DIR)/js_node.c

UI_SOURCES = $(UI_SRC_DIR)/ws_server.c \
             $(UI_SRC_DIR)/sim_state.c

QUICKJS_SRC_DIR = lib/quickjs
QUICKJS_BUILD_DIR = build/quickjs

QUICKJS_SOURCES = $(QUICKJS_SRC_DIR)/quickjs.c \
                  $(QUICKJS_SRC_DIR)/cutils.c \
                  $(QUICKJS_SRC_DIR)/libbf.c \
                  $(QUICKJS_SRC_DIR)/libregexp.c \
                  $(QUICKJS_SRC_DIR)/libunicode.c

QUICKJS_OBJECTS = $(patsubst $(QUICKJS_SRC_DIR)/%.c, $(QUICKJS_BUILD_DIR)/%.o, $(QUICKJS_SOURCES))

LIB_SOURCES = $(LIB_SRC_DIR)/cJSON.c \
              $(LIB_SRC_DIR)/cbor.c

ifneq ($(LIGHTNING_LIBS),)
  CFLAGS += $(LIGHTNING_CFLAGS) -DHAVE_LIGHTNING
  LDFLAGS += $(LIGHTNING_LIBS)
  SOURCES += $(MSP430_SRC_DIR)/msp430_jit.c
endif

COMMON_OBJECTS = $(patsubst $(COMMON_SRC_DIR)/%.c, $(COMMON_BUILD_DIR)/%.o, $(COMMON_SOURCES))
SIM_OBJECTS = $(patsubst $(SIM_SRC_DIR)/%.c, $(SIM_BUILD_DIR)/%.o, $(SIM_SOURCES))
SERVICES_OBJECTS = $(patsubst $(SERVICES_SRC_DIR)/%.c, $(SERVICES_BUILD_DIR)/%.o, $(SERVICES_SOURCES))
MOTES_OBJECTS = $(patsubst $(MOTES_SRC_DIR)/%.c, $(MOTES_BUILD_DIR)/%.o, $(MOTES_SOURCES))
OBJECTS = $(patsubst $(MSP430_SRC_DIR)/%.c, $(MSP430_BUILD_DIR)/%.o, $(SOURCES))
ARM_OBJECTS = $(patsubst $(ARM_SRC_DIR)/%.c, $(ARM_BUILD_DIR)/%.o, $(ARM_SOURCES))
NATIVE_OBJECTS = $(patsubst $(NATIVE_SRC_DIR)/%.c, $(NATIVE_BUILD_DIR)/%.o, $(NATIVE_SOURCES))
UI_OBJECTS = $(patsubst $(UI_SRC_DIR)/%.c, $(UI_BUILD_DIR)/%.o, $(UI_SOURCES))
LIB_OBJECTS = $(patsubst $(LIB_SRC_DIR)/%.c, $(LIB_BUILD_DIR)/%.o, $(LIB_SOURCES))

TEST_SOURCES = $(TEST_DIR)/test_main.c \
               $(TEST_DIR)/test_correctness.c \
               $(TEST_DIR)/test_benchmark.c \
               $(TEST_DIR)/test_firmware.c \
               $(TEST_DIR)/test_arm_correctness.c \
               $(TEST_DIR)/test_arm_firmware.c \
               $(TEST_DIR)/test_mixed_multinode.c \
               $(TEST_DIR)/test_timeline.c \
               $(TEST_DIR)/test_mock_host.c \
               $(TEST_DIR)/test_cc1200.c \
               $(TEST_DIR)/test_radio_medium.c \
               $(TEST_DIR)/test_radio_bus.c

TEST_OBJECTS = $(patsubst $(TEST_DIR)/%.c, $(BUILD_DIR)/test_%.o, $(TEST_SOURCES))

# Read CONTIKI_DIR from csim.conf if not set via env/cmdline
CONTIKI_DIR ?= $(shell grep -s '^CONTIKI_DIR=' csim.conf | cut -d= -f2-)
ifeq ($(CONTIKI_DIR),)
  CONTIKI_DIR = ../contiki-ng
endif

.PHONY: all clean test bench test-firmware test-arm cooja-tests chain-tests build-firmware configure

all: $(BUILD_DIR)/test_runner

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(MSP430_BUILD_DIR):
	mkdir -p $(MSP430_BUILD_DIR)

$(ARM_BUILD_DIR):
	mkdir -p $(ARM_BUILD_DIR)

$(NATIVE_BUILD_DIR):
	mkdir -p $(NATIVE_BUILD_DIR)

$(UI_BUILD_DIR):
	mkdir -p $(UI_BUILD_DIR)

$(COMMON_BUILD_DIR):
	mkdir -p $(COMMON_BUILD_DIR)

$(SIM_BUILD_DIR):
	mkdir -p $(SIM_BUILD_DIR)

$(SERVICES_BUILD_DIR):
	mkdir -p $(SERVICES_BUILD_DIR)

$(MOTES_BUILD_DIR):
	mkdir -p $(MOTES_BUILD_DIR)

$(LIB_BUILD_DIR):
	mkdir -p $(LIB_BUILD_DIR)

$(QUICKJS_BUILD_DIR):
	mkdir -p $(QUICKJS_BUILD_DIR)

$(COMMON_BUILD_DIR)/%.o: $(COMMON_SRC_DIR)/%.c | $(COMMON_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(SIM_BUILD_DIR)/%.o: $(SIM_SRC_DIR)/%.c | $(SIM_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(SERVICES_BUILD_DIR)/%.o: $(SERVICES_SRC_DIR)/%.c | $(SERVICES_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(MOTES_BUILD_DIR)/%.o: $(MOTES_SRC_DIR)/%.c | $(MOTES_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(MSP430_BUILD_DIR)/%.o: $(MSP430_SRC_DIR)/%.c | $(MSP430_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(ARM_BUILD_DIR)/%.o: $(ARM_SRC_DIR)/%.c | $(ARM_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(NATIVE_BUILD_DIR)/%.o: $(NATIVE_SRC_DIR)/%.c | $(NATIVE_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(UI_BUILD_DIR)/%.o: $(UI_SRC_DIR)/%.c | $(UI_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(LIB_BUILD_DIR)/%.o: $(LIB_SRC_DIR)/%.c | $(LIB_BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# QuickJS needs relaxed warnings (upstream code)
$(QUICKJS_BUILD_DIR)/%.o: $(QUICKJS_SRC_DIR)/%.c | $(QUICKJS_BUILD_DIR)
	$(CC) -O2 -std=c11 -D_GNU_SOURCE -I lib/quickjs -DCONFIG_VERSION=\"2024-01-13\" -w -c $< -o $@

$(BUILD_DIR)/test_%.o: $(TEST_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/test_runner: $(COMMON_OBJECTS) $(SIM_OBJECTS) $(SERVICES_OBJECTS) $(MOTES_OBJECTS) $(OBJECTS) $(ARM_OBJECTS) $(NATIVE_OBJECTS) $(UI_OBJECTS) $(LIB_OBJECTS) $(QUICKJS_OBJECTS) $(TEST_OBJECTS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Auto-generated header dependencies (from -MMD). Catches the case where
# editing a header doesn't trigger a rebuild of every TU that includes
# it — bit us once with nrf54l_radio_state_t shifting the ficr offset
# inside nrf54l15_soc_t after a field add, leaving test_mixed_multinode.o
# writing per-node ficr at the OLD struct offset while the runtime read
# handler dereferenced the NEW one (returned zeros). -include suppresses
# the warning on the first build before any .d files exist.
-include $(shell find $(BUILD_DIR) -name '*.d' 2>/dev/null)

test: $(BUILD_DIR)/test_runner
	./$(BUILD_DIR)/test_runner correctness -v

bench: $(BUILD_DIR)/test_runner
	./$(BUILD_DIR)/test_runner bench

test-firmware: $(BUILD_DIR)/test_runner
	./$(BUILD_DIR)/test_runner firmware

test-arm: $(BUILD_DIR)/test_runner
	./$(BUILD_DIR)/test_runner arm-correctness -v

clean:
	rm -rf $(BUILD_DIR)

# Run Contiki-NG Cooja test suite
cooja-tests: $(BUILD_DIR)/test_runner
	CONTIKI_DIR=$(CONTIKI_DIR) ./tools/run-cooja-tests.sh $(PATTERN) $(if $(VERBOSE),-v)

# Run per-platform RPL-UDP chain tests (one per platform with a radio).
# Usage:
#   make chain-tests
#   make chain-tests PLATFORM=sky
#   make chain-tests PLATFORM="sky cc2538dk"
chain-tests: $(BUILD_DIR)/test_runner
	./tools/run-chain-tests.sh $(PLATFORM)

# Build firmware for Cooja tests
build-firmware:
	CONTIKI_DIR=$(CONTIKI_DIR) ./tools/build-test-firmware.sh --target cooja $(PATTERN)

# Write csim.conf with CONTIKI_DIR
configure:
	@echo "CONTIKI_DIR=$(CONTIKI_DIR)" > csim.conf
	@echo "Wrote csim.conf: CONTIKI_DIR=$(CONTIKI_DIR)"

# Debug build
debug: CFLAGS = -O0 -g -Wall -Wextra -Wno-unused-parameter -std=c11 -I include/common -I include/sim -I include/msp430 -I include/arm -I include/native -I include/ui -I lib -I lib/quickjs -DDEBUG
debug: LDFLAGS = -lm -lpthread
debug: clean $(BUILD_DIR)/test_runner

# Profile-guided optimization (Apple Clang)
PGO_CFLAGS = -O3 -std=c11 -I include/common -I include/sim -I include/msp430 -I include/arm -I include/native -I include/ui -I lib -I lib/quickjs -march=native
PGO_LDFLAGS = -lm -lpthread
ifneq ($(LIGHTNING_LIBS),)
  PGO_CFLAGS += $(LIGHTNING_CFLAGS) -DHAVE_LIGHTNING
  PGO_LDFLAGS += $(LIGHTNING_LIBS)
endif

pgo:
	@echo "=== PGO Step 1: Instrumented build ==="
	rm -rf $(BUILD_DIR)
	mkdir -p $(BUILD_DIR)
	$(CC) $(PGO_CFLAGS) -fprofile-instr-generate \
		$(COMMON_SOURCES) $(SOURCES) $(ARM_SOURCES) $(NATIVE_SOURCES) $(UI_SOURCES) $(LIB_SOURCES) $(TEST_SOURCES) -o $(BUILD_DIR)/test_runner $(PGO_LDFLAGS)
	@echo "=== PGO Step 2: Collecting profile data ==="
	LLVM_PROFILE_FILE="$(BUILD_DIR)/default.profraw" ./$(BUILD_DIR)/test_runner bench
	LLVM_PROFILE_FILE="$(BUILD_DIR)/default.profraw" ./$(BUILD_DIR)/test_runner correctness
	xcrun llvm-profdata merge -output=$(BUILD_DIR)/default.profdata $(BUILD_DIR)/default.profraw
	@echo "=== PGO Step 3: Optimized build ==="
	$(CC) $(CFLAGS) -fprofile-instr-use=$(BUILD_DIR)/default.profdata \
		$(COMMON_SOURCES) $(SOURCES) $(ARM_SOURCES) $(NATIVE_SOURCES) $(UI_SOURCES) $(LIB_SOURCES) $(TEST_SOURCES) -o $(BUILD_DIR)/test_runner $(LDFLAGS)
	@echo "=== PGO build complete ==="
