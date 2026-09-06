/*
 * ARM platform — SoC-agnostic skeleton.
 *
 * Holds the CPU + NVIC + SysTick that every Cortex-M variant shares,
 * plus a `sim_host_t` vtable for off-SoC chip drivers and an
 * `arm_soc_ops_t` vtable that abstracts SoC-specific peripheral
 * lifecycle (init / destroy / console hookup).
 *
 * The actual SoC peripherals live behind `plat->soc`, allocated by the
 * SoC's `init` op. CC2538 peripherals live in `cc2538_soc_t` (see
 * `cc2538_soc.h`); future SoCs (nRF52840, ...) plug in here without
 * changing this header.
 */
#ifndef ARM_PLATFORM_H
#define ARM_PLATFORM_H

#include "arm_cpu.h"
#include "arm_config.h"
#include "arm_nvic.h"
#include "arm_systick.h"
#include "sim_host.h"

/* UART TX callback */
typedef void (*arm_uart_tx_callback)(void *user_data, uint8_t byte);

/* Optional GPIO pin descriptor (port 0..3 = A..D, pin 0..7).
 * Used by platform configs to document board wiring (LEDs, buttons,
 * off-SoC chip control pins). port < 0 means "unused/not present". */
typedef struct arm_gpio_pin {
    int8_t  port;         /* 0=A, 1=B, 2=C, 3=D, -1=unused */
    int8_t  pin;          /* 0..7 */
    bool    active_low;   /* true if asserted = drive low */
} arm_gpio_pin_t;

/* Forward declarations for the SoC-ops vtable. */
struct arm_platform;
struct arm_platform_config;

/* SoC-specific lifecycle vtable. One instance per SoC family
 * (cc2538_soc_ops, nrf52840_soc_ops, …). Installed via
 * `arm_platform_config_t::soc_ops`.
 *
 * The init op owns:
 *   - allocating `plat->soc` (and the peripheral state inside it),
 *   - constructing the per-peripheral state (UART/GPIO/timer/radio/…),
 *   - populating `plat->host` with pointers to the SoC's GPIO etc.,
 *   - installing any off-SoC chip wiring described by the platform
 *     config (e.g. CC1200 on Firefly).
 *
 * The destroy op tears down whatever init allocated. The set_console
 * op is split out because it may be called outside init (e.g. when the
 * test harness wires a per-node UART callback after platform init). */
typedef struct arm_soc_ops {
    const char *name;             /* "cc2538", "nrf52840", … */
    void   (*init)(struct arm_platform *plat);
    void   (*destroy)(struct arm_platform *plat);
    void   (*set_console)(struct arm_platform *plat,
                          arm_uart_tx_callback cb, void *user_data);
} arm_soc_ops_t;

/* Static platform configuration */
typedef struct arm_platform_config {
    const char          *name;
    const arm_config_t  *soc;          /* SoC memory map / IRQ count / freq */
    const arm_soc_ops_t *soc_ops;      /* SoC peripheral lifecycle vtable */
    int                  console_uart; /* 0 or 1 (for SoCs with multiple UARTs) */
    /* Board wiring metadata. Optional — zero-initialized entries are
     * treated as "not described". Currently informational only; the
     * underlying GPIO peripheral is wired identically for all boards
     * on a given SoC. Useful once a board needs platform-managed LED
     * state or output-pin callback fan-out. */
    arm_gpio_pin_t       leds[3];      /* up to 3 status LEDs */
    arm_gpio_pin_t       button;       /* user button */
    /* Off-SoC CC1200 sub-GHz radio wiring. Set has_cc1200=true to enable;
     * leave zero-initialized to skip CC1200 init entirely. CC2538-only
     * for now; if a future non-CC2538 board adds a CC1200, move this
     * block into a per-SoC sub-config. */
    bool                 has_cc1200;
    int                  cc1200_ssi;       /* 0 or 1 — which SSI bus  */
    arm_gpio_pin_t       cc1200_csn;       /* chip-select (active low) */
    arm_gpio_pin_t       cc1200_reset;     /* reset       (active low) */
    arm_gpio_pin_t       cc1200_gdo0;      /* GDO0        (input to MCU) */
    arm_gpio_pin_t       cc1200_gdo2;      /* GDO2        (input to MCU, optional) */
    /* Off-SoC SPI chips the board ships with (nRF54L15-DK: MX25R6435F on
     * SPIM00/P2.05, ENC28J60 on SPIM22/P1.12).  Attached by the SoC init
     * unless a node's config "peripherals" list replaces them.  A NULL
     * chip name terminates the list. */
    struct {
        const char *chip;     /* "mx25r6435f", "enc28j60", … */
        int         spim;     /* SPIM instance id */
        int         cs_port, cs_pin;
    } spi_chips[4];
    /* Per-board VTOR override. 0 → use SoC's `arm_config_t::vtor_default`
     * (and fall back to flash_base / CCA discovery). Non-zero → that
     * absolute address. Lets two boards with the same SoC differ on
     * whether the application starts at flash_base or above a
     * bootloader region (e.g. nrf52840 DK at 0x0 vs Dongle at 0x1000). */
    uint32_t             vtor_override;
} arm_platform_config_t;

/* Platform runtime state. SoC-agnostic — the SoC-specific peripheral
 * bundle lives behind `soc` and is reached via the SoC's accessor
 * (e.g. `arm_platform_cc2538(plat)`). */
typedef struct arm_platform {
    arm_cpu_t           cpu;
    arm_nvic_t          nvic;
    arm_systick_t       systick;
    /* CPU-agnostic vtable for off-SoC chip drivers (CC1200, external
     * CC2420, etc.). Populated by the SoC's init op once GPIO is up. */
    sim_host_t          host;
    /* SoC-specific peripheral state. Allocated by `soc_ops->init`,
     * freed by `soc_ops->destroy`. */
    void               *soc;
    const arm_platform_config_t *config;
} arm_platform_t;

/* Initialize all peripherals from platform config */
void arm_platform_init(arm_platform_t *plat, const arm_platform_config_t *config);

/* Clean up */
void arm_platform_destroy(arm_platform_t *plat);

/* Connect a TX callback to the console UART */
void arm_platform_set_console(arm_platform_t *plat,
                              arm_uart_tx_callback cb, void *user_data);

/* Lookup platform by name */
const arm_platform_config_t *arm_platform_find(const char *name);

#endif /* ARM_PLATFORM_H */
