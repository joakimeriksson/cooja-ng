/*
 * nrf54l15_spim — SPIM (SPI master with EasyDMA) peripheral model for
 * the nRF54L15.
 *
 * Register offsets and instance parameters come from the nrfx MDK
 * (nrf54l15_types.h NRF_SPIM_Type, nrf54l15_application_peripherals.h
 * SPIMxx_* feature macros, nrf54l15_global.h NRF_SPIMxx_S_BASE) — not
 * from the product spec and not guessed.  The nRF54L SPIM uses the
 * "DMA register" layout (NRF_SPIM_HAS_DMA_REG in hal/nrf_spim.h): the
 * buffers live under DMA.TX / DMA.RX at 0x700, and the per-direction
 * end events under EVENTS_DMA.{RX,TX}.END at 0x14C / 0x168.
 *
 * Scope: what nrfx_spim in *blocking* mode (no event handler) touches.
 * That is exactly the Contiki-NG arch/cpu/nrf/dev/spi-arch.c path:
 *
 *   nrfx_spim_init   → PSEL.SCK/MOSI/MISO, ORC, PRESCALER, CONFIG,
 *                      IFTIMING.RXDELAY, (CSN/DCX pins unused)
 *   nrfx_spim_xfer   → DMA.TX.PTR/MAXCNT, DMA.RX.PTR/MAXCNT,
 *                      EVENTS_END = 0, ENABLE = 7, TASKS_START = 1,
 *                      poll EVENTS_END, read DMA.*.AMOUNT,
 *                      TASKS_STOP = 1, poll EVENTS_STOPPED, ENABLE = 0
 *   nrfx_spim_uninit → INTENCLR
 *
 * Interrupts are modelled (INTENSET/INTENCLR + an optional irq hook)
 * but the blocking driver never enables them.
 *
 * The model is CPU-agnostic on purpose: time comes from a sim_host_t,
 * EasyDMA memory from two byte accessors, and the wire from an
 * exchange callback.  That lets test/test_nrf54l15_spim.c program the
 * block through the same read/write entry points the MMIO dispatcher
 * uses, with a mock host and a plain byte array as "RAM".
 */
#ifndef NRF54L15_SPIM_H
#define NRF54L15_SPIM_H

#include <stdint.h>
#include <stdbool.h>
#include "sim_host.h"

/* --- Instance base addresses (nrf54l15_global.h, secure alias) ------- */
#define NRF54L_SPIM00_BASE   0x5004A000u
#define NRF54L_SPIM20_BASE   0x500C6000u   /* SERIAL20 — the console UARTE20 slot */
#define NRF54L_SPIM21_BASE   0x500C7000u
#define NRF54L_SPIM22_BASE   0x500C8000u
#define NRF54L_SPIM30_BASE   0x50104000u
/* NRF_SPIM_Type is 0x75C bytes, but nrfx's nRF54L errata 55 / 8 work-
 * arounds poke +0xC80 / +0xC84, so the region swallows the whole 4 KiB. */
#define NRF54L_SPIM_SIZE     0x1000u

/* --- Register offsets (nrf54l15_types.h NRF_SPIM_Type) ---------------- */
#define SPIM_TASKS_START          0x000
#define SPIM_TASKS_STOP           0x004
#define SPIM_TASKS_SUSPEND        0x00C
#define SPIM_TASKS_RESUME         0x010
#define SPIM_SUBSCRIBE_START      0x080
#define SPIM_SUBSCRIBE_STOP       0x084
#define SPIM_EVENTS_STARTED       0x100
#define SPIM_EVENTS_STOPPED       0x104
#define SPIM_EVENTS_END           0x108
#define SPIM_EVENTS_DMA_RX_END    0x14C
#define SPIM_EVENTS_DMA_RX_READY  0x150
#define SPIM_EVENTS_DMA_RX_BUSERR 0x154
#define SPIM_EVENTS_DMA_TX_END    0x168
#define SPIM_EVENTS_DMA_TX_READY  0x16C
#define SPIM_EVENTS_DMA_TX_BUSERR 0x170
#define SPIM_PUBLISH_STARTED      0x180
#define SPIM_PUBLISH_STOPPED      0x184
#define SPIM_PUBLISH_END          0x188
#define SPIM_SHORTS               0x200
#define SPIM_INTENSET             0x304
#define SPIM_INTENCLR             0x308
#define SPIM_ENABLE               0x500
#define SPIM_PRESCALER            0x52C
#define SPIM_CONFIG               0x554
#define SPIM_IFTIMING_RXDELAY     0x5AC
#define SPIM_IFTIMING_CSNDUR      0x5B0
#define SPIM_DCXCNT               0x5B4
#define SPIM_CSNPOL               0x5B8
#define SPIM_ORC                  0x5C0
#define SPIM_PSEL_SCK             0x600
#define SPIM_PSEL_MOSI            0x604
#define SPIM_PSEL_MISO            0x608
#define SPIM_PSEL_DCX             0x60C
#define SPIM_PSEL_CSN             0x610
#define SPIM_DMA_RX_PTR           0x704
#define SPIM_DMA_RX_MAXCNT        0x708
#define SPIM_DMA_RX_AMOUNT        0x70C
#define SPIM_DMA_RX_LIST          0x714
#define SPIM_DMA_RX_TERMONBUSERR  0x71C
#define SPIM_DMA_RX_BUSERRADDR    0x720
#define SPIM_DMA_RX_MATCH_CONFIG  0x724
#define SPIM_DMA_TX_PTR           0x73C
#define SPIM_DMA_TX_MAXCNT        0x740
#define SPIM_DMA_TX_AMOUNT        0x744
#define SPIM_DMA_TX_LIST          0x74C
#define SPIM_DMA_TX_TERMONBUSERR  0x754
#define SPIM_DMA_TX_BUSERRADDR    0x758

/* --- Bit fields --------------------------------------------------------- */
#define SPIM_ENABLE_ENABLED       0x7u
#define SPIM_PRESCALER_MSK        0x7Fu
#define SPIM_PRESCALER_RESET      0x40u
#define SPIM_MAXCNT_MSK           0xFFFFu
#define SPIM_CONFIG_ORDER_LSB     (1u << 0)
#define SPIM_CONFIG_CPHA          (1u << 1)
#define SPIM_CONFIG_CPOL          (1u << 2)
#define SPIM_SHORTS_END_START     (1u << 17)
#define SPIM_INT_STARTED          (1u << 0)
#define SPIM_INT_STOPPED          (1u << 1)
#define SPIM_INT_END              (1u << 2)
#define SPIM_INT_DMARXEND         (1u << 19)
#define SPIM_INT_DMARXREADY       (1u << 20)
#define SPIM_INT_DMATXEND         (1u << 26)
#define SPIM_INT_DMATXREADY       (1u << 27)
#define SPIM_PSEL_DISCONNECTED    0xFFFFFFFFu

/* One byte on the wire: MOSI in, MISO out.  The platform installs a
 * router that picks the chip whose chip-select is currently asserted;
 * with nothing selected the bus floats high and the callback returns
 * 0xFF (or the callback is NULL and the model does). */
typedef uint8_t (*nrf54l_spim_exchange_fn)(void *user, int instance, uint8_t mosi);

typedef struct nrf54l_spim {
    const sim_host_t *host;
    int      instance;           /* 0, 20, 21, 22, 30 — the id in NRF_SPIMxx */
    uint32_t base_freq_hz;       /* SPIM00: 128 MHz core; others: 16 MHz */
    uint32_t prescaler_min;      /* SPIM00: 4; others: 2 (MDK DIVISOR_RANGE_MIN) */

    /* EasyDMA memory access, bound by the platform (arm_read8/write8)
     * or by a unit test (byte array). */
    uint8_t (*mem_read8)(void *mem, uint32_t addr);
    void    (*mem_write8)(void *mem, uint32_t addr, uint8_t value);
    void    *mem;

    /* Wire + optional IRQ sink (NVIC pending on the SERIALxx line). */
    nrf54l_spim_exchange_fn exchange;
    void    *exchange_user;
    void   (*irq)(void *irq_user);
    void    *irq_user;

    /* Register file — only what the driver reads back. */
    uint32_t enable;
    uint32_t prescaler;
    uint32_t config;
    uint32_t orc;
    uint32_t shorts;
    uint32_t inten;
    uint32_t psel_sck, psel_mosi, psel_miso, psel_dcx, psel_csn;
    uint32_t rxdelay, csndur, dcxcnt, csnpol;
    uint32_t tx_ptr, tx_maxcnt, tx_amount, tx_list;
    uint32_t rx_ptr, rx_maxcnt, rx_amount, rx_list;
    uint32_t evt_started, evt_stopped, evt_end;
    uint32_t evt_rx_end, evt_rx_ready, evt_tx_end, evt_tx_ready;
    uint32_t publish_started, publish_stopped, publish_end;
    uint32_t subscribe_start, subscribe_stop;

    /* Transfer in flight: START latched the lengths and armed
     * xfer_event at now + bytes * 8 / bit-rate. */
    bool        busy;
    uint32_t    xfer_len;
    cpu_event_t xfer_event;

    /* Diagnostics */
    uint64_t stat_transfers;
    uint64_t stat_bytes;
} nrf54l_spim_t;

/* Reset the block and bind its host.  `instance` selects the clock
 * parameters (0 → fast domain).  Memory / wire / irq hooks are set by
 * assigning the struct fields afterwards. */
void nrf54l_spim_init(nrf54l_spim_t *s, const sim_host_t *host, int instance);

/* MMIO entry points.  `off` is the byte offset from the instance base. */
uint32_t nrf54l_spim_read(nrf54l_spim_t *s, uint32_t off);
void     nrf54l_spim_write(nrf54l_spim_t *s, uint32_t off, uint32_t value);

/* Effective SCK rate for the current PRESCALER (base / max(div, min)). */
uint32_t nrf54l_spim_bit_rate_hz(const nrf54l_spim_t *s);

/* Nanoseconds a transfer of `nbytes` occupies on the wire at the
 * current bit rate — the delay between TASKS_START and EVENTS_END. */
int64_t  nrf54l_spim_transfer_ns(const nrf54l_spim_t *s, uint32_t nbytes);

#endif /* NRF54L15_SPIM_H */
