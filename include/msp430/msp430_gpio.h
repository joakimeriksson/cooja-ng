/*
 * GPIO port — stores direction/output/input/interrupt registers.
 *
 * Port 1/2 (interrupt-capable): IN, OUT, DIR, IFG, IES, IE, SEL (7 regs)
 * Port 3-6 (non-interrupt):     IN, OUT, DIR, SEL (4 regs)
 */
#ifndef MSP430_GPIO_H
#define MSP430_GPIO_H

#include "msp430_cpu.h"
#include "msp430_config.h"

#define MSP430_GPIO_MAX_PORTS 10

typedef struct msp430_gpio_port {
    uint8_t  in;
    uint8_t  out;
    uint8_t  dir;
    uint8_t  ifg;           /* Interrupt Flag (P1/P2 only) */
    uint8_t  ies;           /* Interrupt Edge Select (P1/P2 only) */
    uint8_t  ie;            /* Interrupt Enable (P1/P2 only) */
    uint8_t  sel;
    uint32_t base_addr;     /* base address for this port's registers */
    int      interrupt_vector;  /* -1 = no interrupt capability */
    bool     has_interrupt;
} msp430_gpio_port_t;

/* Callback when any port's OUT register changes */
typedef void (*gpio_output_callback_fn)(void *user_data, int port, uint8_t old_out, uint8_t new_out);

typedef struct msp430_gpio {
    msp430_cpu_t       *cpu;
    msp430_gpio_port_t  ports[MSP430_GPIO_MAX_PORTS];
    int                 num_ports;
    gpio_output_callback_fn output_callback;
    void                   *output_callback_data;
} msp430_gpio_t;

/* Initialize GPIO and register a single port */
void msp430_gpio_init(msp430_gpio_t *gpio, msp430_cpu_t *cpu);
void msp430_gpio_add_port(msp430_gpio_t *gpio, int port_num, uint32_t base_addr,
                           int interrupt_vector);

/* Initialize all GPIO ports from MCU config */
void msp430_gpio_init_from_config(msp430_gpio_t *gpio, msp430_cpu_t *cpu,
                                   const msp430_config_t *config);

/* Register callback for output pin changes */
void msp430_gpio_set_output_callback(msp430_gpio_t *gpio,
                                      gpio_output_callback_fn cb, void *user_data);

/* Set/clear a single input pin (used by peripherals to drive input signals).
 * Triggers port interrupt if edge matches IES and pin is enabled in IE. */
void msp430_gpio_set_input_pin(msp430_gpio_t *gpio, int port, int pin, bool value);

/* Update port interrupt status — exposed so peripherals can force an
 * IFG check after touching IE/IFG directly. */
void msp430_gpio_update_interrupt(msp430_gpio_t *gpio, int port_idx);

/* Force-enable IE and pulse IFG on a pin to emulate legacy Cooja firmware
 * behavior for FIFOP (CC2420). Sets IE for the pin and either sets (rising)
 * or clears (falling) IFG, then re-checks the interrupt. This abstracts
 * direct port-state pokes that previously lived in chip drivers. */
void msp430_gpio_force_irq_edge(msp430_gpio_t *gpio, int port, int pin, bool rising);

#endif /* MSP430_GPIO_H */
