/*
 * Built-in board registry — see include/sim/sim_board.h.
 *
 * One row per supported board.  The table is intentionally static
 * (refactor plan §7.1: static registry first, dlopen plugins only after
 * the API is stable).  Rows preserve the runner's historical extension
 * behavior exactly, including the ".nrf52840" → nrf52840-dk alias
 * (firmware built with the default BOARD=dk) and the Tmote Sky default
 * for unknown extensions.
 */
#include "sim_board.h"

#include <stddef.h>
#include <string.h>

static const sim_board_desc_t boards[] = {
    /* MSP430 family */
    { ".sky",          "sky",             SIM_BOARD_KIND_MSP430, "MSP430/Sky"      },
    { ".z1",           "z1",              SIM_BOARD_KIND_MSP430, "MSP430/Z1"       },
    { ".esb",          "esb",             SIM_BOARD_KIND_MSP430, "MSP430/ESB"      },
    { ".wismote",      "wismote",         SIM_BOARD_KIND_MSP430, "MSP430/WisMote"  },
    { ".exp5438",      "exp5438",         SIM_BOARD_KIND_MSP430, "MSP430/exp5438"  },
    { ".msp430fr5969", "fr5969",          SIM_BOARD_KIND_MSP430, "MSP430/FR5969"   },
    /* ARM family.  (Labels are accurate per board now — the old runner
     * banner said "ARM/CC2538DK" for every ARM board.) */
    { ".cc2538dk",        "cc2538dk",        SIM_BOARD_KIND_ARM, "ARM/CC2538DK"        },
    { ".zoul-firefly",    "zoul-firefly",    SIM_BOARD_KIND_ARM, "ARM/Zoul-Firefly"    },
    { ".nrf52840-dongle", "nrf52840-dongle", SIM_BOARD_KIND_ARM, "ARM/nRF52840-Dongle" },
    { ".nrf52840-dk",     "nrf52840-dk",     SIM_BOARD_KIND_ARM, "ARM/nRF52840-DK"     },
    /* .nrf52840 without board suffix: built with BOARD=dk (the default). */
    { ".nrf52840",        "nrf52840-dk",     SIM_BOARD_KIND_ARM, "ARM/nRF52840-DK"     },
    { ".nrf54l15-dk",     "nrf54l15-dk",     SIM_BOARD_KIND_ARM, "ARM/nRF54L15-DK"     },
    /* Host-process motes */
    { ".cooja", NULL, SIM_BOARD_KIND_NATIVE, "Native/Cooja" },
    { ".js",    NULL, SIM_BOARD_KIND_JS,     "JS/QuickJS"   },
};

#define BOARD_COUNT ((int)(sizeof(boards) / sizeof(boards[0])))

/* Historical default: unknown extension → Tmote Sky. */
static const sim_board_desc_t *default_board = &boards[0];

const sim_board_desc_t *sim_board_for_path(const char *path) {
    if (!path) return default_board;
    const char *dot = strrchr(path, '.');
    if (!dot) return default_board;
    for (int i = 0; i < BOARD_COUNT; i++) {
        if (strcmp(dot, boards[i].extension) == 0)
            return &boards[i];
    }
    return default_board;
}

const sim_board_desc_t *sim_board_find(const char *name) {
    if (!name) return NULL;
    for (int i = 0; i < BOARD_COUNT; i++) {
        if (boards[i].name && strcmp(name, boards[i].name) == 0)
            return &boards[i];
    }
    return NULL;
}
