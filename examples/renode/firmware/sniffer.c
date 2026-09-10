/*
 * csim-sniffer — bare-metal Cortex-M firmware for Renode that watches a
 * csim 802.15.4 network.
 *
 * Runs inside Renode, reads csim's co-simulation register window, and
 * prints every frame the csim medium delivers to it: who sent it, how
 * strong it was here, when it started on the air, and what it is.  Renode
 * drives csim's clock the whole time, so the timestamps it prints are
 * csim's simulation time, not wall-clock.
 *
 * Interrupt-driven, and it has to be: every access to the window is a
 * synchronous round trip to the csim process, so a tight polling loop would
 * cost millions of them per simulated second.  Here the CPU sits in WFI and
 * touches the bus only when csim raises the device's line.  SysTick gives a
 * second, slower wake-up, which paces the status line and doubles as a
 * safety net if the device interrupt never arrives.
 *
 * Build:  make -C examples/renode/firmware
 * Run:    see examples/renode/README.md
 */
#include <stdint.h>

#define CSIM_DEV_BASE 0x40100000UL
#include "../csim_dev.h"

/* PL011 UART, as mapped by csim-host.repl. */
#define UART_BASE 0x40000000UL
#define UART_DR   (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_FR   (*(volatile uint32_t *)(UART_BASE + 0x18))
#define UART_CR   (*(volatile uint32_t *)(UART_BASE + 0x30))
#define UART_FR_TXFF (1U << 5)
#define UART_CR_UARTEN (1U << 0)
#define UART_CR_TXE    (1U << 8)
#define UART_CR_RXE    (1U << 9)

static void uart_init(void) {
    UART_CR = UART_CR_UARTEN | UART_CR_TXE | UART_CR_RXE;
}

static void uart_putc(char c) {
    /* Bounded wait: a UART model that never clears TXFF would otherwise
     * hang the demo with no output to explain why. */
    for (int spins = 0; spins < 10000; spins++) {
        if (!(UART_FR & UART_FR_TXFF)) break;
    }
    UART_DR = (uint32_t)(unsigned char)c;
}

static void print(const char *s) {
    while (*s) uart_putc(*s++);
}

static void print_u32(uint32_t v) {
    char buf[11];
    int i = 10;
    buf[i--] = '\0';
    if (v == 0) buf[i--] = '0';
    while (v) { buf[i--] = (char)('0' + (v % 10)); v /= 10; }
    print(&buf[i + 1]);
}

static void print_i32(int32_t v) {
    if (v < 0) { uart_putc('-'); v = -v; }
    print_u32((uint32_t)v);
}

static void print_hex8(uint8_t v) {
    static const char h[] = "0123456789abcdef";
    uart_putc(h[v >> 4]);
    uart_putc(h[v & 0xf]);
}

/* csim's clock is 64-bit ns; print it as seconds with 6 decimals, which is
 * how csim prints its own timestamps, so the two logs line up. */
static void print_time(uint64_t ns) {
    uint32_t sec = (uint32_t)(ns / 1000000000ULL);
    uint32_t us  = (uint32_t)((ns % 1000000000ULL) / 1000ULL);
    print_u32(sec);
    uart_putc('.');
    for (uint32_t d = 100000; d > 0; d /= 10) {
        uart_putc((char)('0' + (us / d) % 10));
    }
}

/* ---- 802.15.4 ---------------------------------------------------------- */

#define FCF_TYPE(fcf)      ((fcf) & 0x7)
#define FCF_SEC            (1U << 3)
#define FCF_PENDING        (1U << 4)
#define FCF_ACK_REQ        (1U << 5)
#define FCF_PANID_COMPRESS (1U << 6)
#define FCF_DST_MODE(fcf)  (((fcf) >> 10) & 0x3)
#define FCF_VERSION(fcf)   (((fcf) >> 12) & 0x3)
#define FCF_SRC_MODE(fcf)  (((fcf) >> 14) & 0x3)

static const char *frame_type_name(unsigned t) {
    switch (t) {
    case 0: return "BEACON";
    case 1: return "DATA";
    case 2: return "ACK";
    case 3: return "CMD";
    default: return "?";
    }
}

/* Address field width for a mode: 0 = absent, 2 = short, 3 = extended. */
static int addr_len(unsigned mode) {
    return (mode == 2) ? 2 : (mode == 3) ? 8 : 0;
}

static void print_addr(const uint8_t *p, int len) {
    /* 802.15.4 sends addresses little-endian; print most-significant first
     * so it reads like the addresses Contiki logs. */
    for (int i = len - 1; i >= 0; i--) {
        print_hex8(p[i]);
        if (i) uart_putc(':');
    }
}

/*
 * Classify the payload.  Contiki's RPL traffic is IPv6 compressed with
 * 6LoWPAN IPHC, so the ICMPv6 type is not at a fixed offset — this reports
 * the 6LoWPAN dispatch, and for an uncompressed-next-header IPHC frame it
 * looks one step further for the RPL message type.  Enough to tell DIO
 * multicast chatter from DAO/DIS and from application UDP.
 */
static void print_payload_kind(const uint8_t *p, int len) {
    if (len <= 0) { print(" payload=none"); return; }

    uint8_t d = p[0];
    if ((d & 0xE0) == 0x60) {
        /* IPHC (011xxxxx) */
        print(" 6lowpan=IPHC");
        if (len >= 2 && (p[1] & 0x0F) == 0x00) {
            /* No context, no compressed next header: NH byte follows the
             * IPHC header.  The header length varies, so scan a short window
             * for an ICMPv6 RPL control message rather than guess an offset. */
            for (int i = 2; i < len - 1 && i < 24; i++) {
                if (p[i] == 58 /* ICMPv6 */) {
                    /* type byte follows the compressed addresses; look for
                     * 155 (RPL control) nearby */
                    for (int j = i + 1; j < len && j < i + 24; j++) {
                        if (p[j] == 155) {
                            print(" RPL");
                            if (j + 1 < len) {
                                switch (p[j + 1]) {
                                case 0x00: print("-DIS"); break;
                                case 0x01: print("-DIO"); break;
                                case 0x02: print("-DAO"); break;
                                case 0x03: print("-DAO-ACK"); break;
                                default: break;
                                }
                            }
                            return;
                        }
                    }
                }
            }
        }
        return;
    }
    if ((d & 0xF8) == 0xF0) { print(" 6lowpan=FRAG"); return; }
    if (d == 0x41)          { print(" 6lowpan=uncompressed-IPv6"); return; }
    print(" payload=0x");
    print_hex8(d);
}

/* ---- main -------------------------------------------------------------- */

/* SysTick, for a periodic wake-up that costs no bus traffic. */
#define SYST_CSR  (*(volatile uint32_t *)0xE000E010UL)
#define SYST_RVR  (*(volatile uint32_t *)0xE000E014UL)
#define SYST_CVR  (*(volatile uint32_t *)0xE000E018UL)
#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100UL)

#define CSIM_IRQ 10

static uint8_t  frame[CSIM_MAX_FRAME];
static uint32_t rx_total;
static volatile uint32_t irq_hits;
static volatile uint32_t ticks;

/*
 * The device's line is level-triggered: it stays high while a frame is
 * queued.  Returning from the handler with it still high would re-enter
 * immediately, so the handler masks the source and main re-arms it once the
 * queue has been drained.
 */
void csim_irq_handler(void) {
    CSIM_CTRL = 0;
    irq_hits++;
}

void systick_handler(void) {
    ticks++;
}

static void drain(void) {
    while (CSIM_RX_COUNT) {
        uint32_t len  = CSIM_RX_LEN;
        int32_t  from = (int32_t)CSIM_RX_FROM;
        int32_t  rssi = (int32_t)CSIM_RX_RSSI;
        uint32_t ch   = CSIM_RX_CHANNEL;
        uint64_t at   = ((uint64_t)CSIM_RX_TIME_HI << 32) | CSIM_RX_TIME_LO;

        uint32_t n = (len < sizeof(frame)) ? len : sizeof(frame);
        for (uint32_t i = 0; i < n; i++)
            frame[i] = (uint8_t)(*(volatile uint8_t *)(CSIM_DEV_BASE + 0x30));
        CSIM_RX_POP = 1;
        rx_total++;

        print("[renode] ");
        print_time(at);
        print(" rx #");
        print_u32(rx_total);
        print(" node=");
        print_i32(from);
        print(" ch=");
        print_u32(ch);
        print(" rssi=");
        print_i32(rssi);
        print("dBm len=");
        print_u32(len);

        if (n >= 3) {
            uint16_t fcf = (uint16_t)(frame[0] | (frame[1] << 8));
            unsigned type = FCF_TYPE(fcf);
            print(" ");
            print(frame_type_name(type));
            print(" seq=");
            print_u32(frame[2]);

            int off = 3;
            unsigned dm = FCF_DST_MODE(fcf), sm = FCF_SRC_MODE(fcf);
            if (dm) {
                off += 2;                       /* dst PAN */
                if (off + addr_len(dm) <= (int)n) {
                    print(" dst=");
                    print_addr(&frame[off], addr_len(dm));
                }
                off += addr_len(dm);
            }
            if (sm) {
                if (!(fcf & FCF_PANID_COMPRESS)) off += 2;  /* src PAN */
                if (off + addr_len(sm) <= (int)n) {
                    print(" src=");
                    print_addr(&frame[off], addr_len(sm));
                }
                off += addr_len(sm);
            }
            if (fcf & FCF_ACK_REQ) print(" ackreq");
            if (type == 1 && off < (int)n)
                print_payload_kind(&frame[off], (int)n - off);
        }
        print("\r\n");
    }
}

int main(void) {
    uart_init();
    print("\r\n[renode] csim sniffer starting\r\n");

    if (!csim_present()) {
        print("[renode] FATAL: no csim device at 0x40100000 "
              "(is csim running with --renode?)\r\n");
        for (;;) { }
    }

    print("[renode] csim device found: version=");
    print_u32(CSIM_VERSION);
    print(" self_id=");
    print_i32((int32_t)CSIM_SELF_ID);
    print(" nodes=");
    print_u32(CSIM_NODE_COUNT);
    print("\r\n");

    /* Join the network the Sky motes are using. */
    CSIM_CHANNEL = 26;
    NVIC_ISER0 = (1U << CSIM_IRQ);

    /* SysTick: a wake-up that needs no bus access.  The exact rate does not
     * matter, only that it is far slower than the CPU and far faster than
     * the report interval. */
    SYST_RVR = 100000 - 1;
    SYST_CVR = 0;
    SYST_CSR = 0x7;                    /* enable | tickint | processor clock */

    CSIM_CTRL = CSIM_CTRL_IRQ_RX_EN;
    print("[renode] listening on channel 26 (interrupt-driven)\r\n");

    uint32_t last_report = 0;
    for (;;) {
        __asm__ volatile ("wfi");

        /* Either source may have woken us; the drain is the same either
         * way, and re-arming after it is what makes the level line usable. */
        drain();
        CSIM_CTRL = CSIM_CTRL_IRQ_RX_EN;

        uint32_t sec = (uint32_t)(csim_sim_time_ns() / 1000000000ULL);
        if (sec >= last_report + 10) {
            last_report = sec;
            print("[renode] t=");
            print_u32(sec);
            print("s frames=");
            print_u32(rx_total);
            print(" irqs=");
            print_u32(irq_hits);
            print(" dropped=");
            print_u32(CSIM_RX_DROPPED);
            print("\r\n");
        }
    }
}
