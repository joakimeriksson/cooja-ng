/*
 * GPIO port — stores IN/OUT/DIR/SEL registers with optional interrupt support.
 *
 * Port 1/2 (interrupt-capable): IN(+0), OUT(+1), DIR(+2), IFG(+3), IES(+4), IE(+5), SEL(+6)
 * Port 3-6 (non-interrupt):     IN(+0), OUT(+1), DIR(+2), SEL(+3)
 */
#include "msp430_gpio.h"
#include <string.h>
#include <stdio.h>

/* Interrupt-capable port register offsets (P1, P2) */
#define INT_OFF_IN   0
#define INT_OFF_OUT  1
#define INT_OFF_DIR  2
#define INT_OFF_IFG  3
#define INT_OFF_IES  4
#define INT_OFF_IE   5
#define INT_OFF_SEL  6
#define INT_REG_SIZE 7

/* Non-interrupt port register offsets (P3-P6) */
#define NOINT_OFF_IN  0
#define NOINT_OFF_OUT 1
#define NOINT_OFF_DIR 2
#define NOINT_OFF_SEL 3
#define NOINT_REG_SIZE 4

/* Forward declarations */
static void gpio_interrupt_handler(void *user_data, int vector);
static void update_interrupt(msp430_gpio_t *gpio, int port_idx);

/* ================================================================
 * IO read/write handlers for interrupt-capable ports (P1, P2)
 * ================================================================ */

static int gpio_int_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_gpio_t *gpio = (msp430_gpio_t *)user_data;
    (void)word; (void)cycles;

    for (int p = 0; p < gpio->num_ports; p++) {
        msp430_gpio_port_t *port = &gpio->ports[p];
        if (!port->has_interrupt || port->base_addr == 0) continue;
        if (addr >= port->base_addr && addr < port->base_addr + INT_REG_SIZE) {
            switch (addr - port->base_addr) {
            case INT_OFF_IN:  return port->in;
            case INT_OFF_OUT: return port->out;
            case INT_OFF_DIR: return port->dir;
            case INT_OFF_IFG: return port->ifg;
            case INT_OFF_IES: return port->ies;
            case INT_OFF_IE:  return port->ie;
            case INT_OFF_SEL: return port->sel;
            }
        }
    }
    return 0;
}

static void gpio_int_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_gpio_t *gpio = (msp430_gpio_t *)user_data;
    (void)word; (void)cycles;

    for (int p = 0; p < gpio->num_ports; p++) {
        msp430_gpio_port_t *port = &gpio->ports[p];
        if (!port->has_interrupt || port->base_addr == 0) continue;
        if (addr >= port->base_addr && addr < port->base_addr + INT_REG_SIZE) {
            switch (addr - port->base_addr) {
            case INT_OFF_IN:
                port->in = (uint8_t)value;
                break;
            case INT_OFF_OUT: {
                uint8_t old_out = port->out;
                port->out = (uint8_t)value;
                if (old_out != port->out && gpio->output_callback) {
                    gpio->output_callback(gpio->output_callback_data,
                                          p + 1, old_out, port->out);
                }
                break;
            }
            case INT_OFF_DIR:
                port->dir = (uint8_t)value;
                break;
            case INT_OFF_IFG:
                port->ifg = (uint8_t)value;
                update_interrupt(gpio, p);
                break;
            case INT_OFF_IES:
                port->ies = (uint8_t)value;
                break;
            case INT_OFF_IE:
                port->ie = (uint8_t)value;
                update_interrupt(gpio, p);
                break;
            case INT_OFF_SEL:
                port->sel = (uint8_t)value;
                break;
            }
            return;
        }
    }
}

/* ================================================================
 * IO read/write handlers for non-interrupt ports (P3-P6)
 * ================================================================ */

static int gpio_read(void *user_data, uint32_t addr, bool word, int64_t cycles) {
    msp430_gpio_t *gpio = (msp430_gpio_t *)user_data;
    (void)word; (void)cycles;

    for (int p = 0; p < gpio->num_ports; p++) {
        msp430_gpio_port_t *port = &gpio->ports[p];
        if (port->has_interrupt || port->base_addr == 0) continue;
        if (addr >= port->base_addr && addr < port->base_addr + NOINT_REG_SIZE) {
            switch (addr - port->base_addr) {
            case NOINT_OFF_IN:  return port->in;
            case NOINT_OFF_OUT: return port->out;
            case NOINT_OFF_DIR: return port->dir;
            case NOINT_OFF_SEL: return port->sel;
            }
        }
    }
    return 0;
}

static void gpio_write(void *user_data, uint32_t addr, int value, bool word, int64_t cycles) {
    msp430_gpio_t *gpio = (msp430_gpio_t *)user_data;
    (void)word; (void)cycles;

    for (int p = 0; p < gpio->num_ports; p++) {
        msp430_gpio_port_t *port = &gpio->ports[p];
        if (port->has_interrupt || port->base_addr == 0) continue;
        if (addr >= port->base_addr && addr < port->base_addr + NOINT_REG_SIZE) {
            switch (addr - port->base_addr) {
            case NOINT_OFF_IN:
                port->in = (uint8_t)value;
                break;
            case NOINT_OFF_OUT: {
                uint8_t old_out = port->out;
                port->out = (uint8_t)value;
                if (old_out != port->out && gpio->output_callback) {
                    gpio->output_callback(gpio->output_callback_data,
                                          p + 1, old_out, port->out);
                }
                break;
            }
            case NOINT_OFF_DIR:
                port->dir = (uint8_t)value;
                break;
            case NOINT_OFF_SEL:
                port->sel = (uint8_t)value;
                break;
            }
            return;
        }
    }
}

/* ================================================================
 * Interrupt management
 * ================================================================ */

/* Update port interrupt state: flag or unflag with CPU */
static void update_interrupt(msp430_gpio_t *gpio, int port_idx) {
    msp430_gpio_port_t *port = &gpio->ports[port_idx];
    if (!port->has_interrupt) return;

    bool pending = (port->ifg & port->ie) != 0;
    msp430_flag_interrupt(gpio->cpu, port->interrupt_vector, gpio,
                          pending ? gpio_interrupt_handler : NULL, pending);
}

/* Called when CPU services the port interrupt — auto-clear highest-priority IFG */
static void gpio_interrupt_handler(void *user_data, int vector) {
    msp430_gpio_t *gpio = (msp430_gpio_t *)user_data;

    /* Find the port for this vector */
    for (int p = 0; p < gpio->num_ports; p++) {
        msp430_gpio_port_t *port = &gpio->ports[p];
        if (!port->has_interrupt) continue;
        if (port->interrupt_vector == vector) {
            /* Clear highest-priority pending IFG bit (lowest pin number) */
            uint8_t ie_ifg = port->ifg & port->ie;
            if (ie_ifg) {
                int bit = ie_ifg & (-ie_ifg);  /* lowest set bit */
                port->ifg &= ~bit;
            }
            update_interrupt(gpio, p);
            return;
        }
    }
}

/* ================================================================
 * Initialization
 * ================================================================ */

void msp430_gpio_init(msp430_gpio_t *gpio, msp430_cpu_t *cpu) {
    memset(gpio, 0, sizeof(*gpio));
    gpio->cpu = cpu;
}

void msp430_gpio_add_port(msp430_gpio_t *gpio, int port_num, uint32_t base_addr,
                           int interrupt_vector) {
    if (port_num < 1 || port_num > MSP430_GPIO_MAX_PORTS) return;
    int idx = port_num - 1;
    msp430_gpio_port_t *port = &gpio->ports[idx];
    port->base_addr = base_addr;
    port->interrupt_vector = interrupt_vector;
    port->has_interrupt = (interrupt_vector >= 0);
    if (port_num > gpio->num_ports) gpio->num_ports = port_num;

    if (port->has_interrupt) {
        msp430_register_io(gpio->cpu, base_addr, INT_REG_SIZE,
                           gpio_int_read, gpio_int_write, gpio);
    } else {
        msp430_register_io(gpio->cpu, base_addr, NOINT_REG_SIZE,
                           gpio_read, gpio_write, gpio);
    }
}

void msp430_gpio_init_from_config(msp430_gpio_t *gpio, msp430_cpu_t *cpu,
                                   const msp430_config_t *config) {
    msp430_gpio_init(gpio, cpu);
    for (int i = 0; i < config->gpio_num_ports; i++) {
        if (config->gpio_ports[i].base_addr != 0) {
            msp430_gpio_add_port(gpio, i + 1, config->gpio_ports[i].base_addr,
                                 config->gpio_ports[i].interrupt_vector);
        }
    }
}

void msp430_gpio_set_output_callback(msp430_gpio_t *gpio,
                                      gpio_output_callback_fn cb, void *user_data) {
    gpio->output_callback = cb;
    gpio->output_callback_data = user_data;
}

/* Set/clear a single input pin and trigger edge-detect interrupt if applicable.
 * Matches Java MSPSim IOPort.setPinState() behavior. */
void msp430_gpio_set_input_pin(msp430_gpio_t *gpio, int port, int pin, bool value) {
    if (port < 1 || port > gpio->num_ports) return;
    int idx = port - 1;
    msp430_gpio_port_t *p = &gpio->ports[idx];
    uint8_t bit = (1 << pin);
    bool old_value = (p->in & bit) != 0;

    /* Update the IN register */
    if (value)
        p->in |= bit;
    else
        p->in &= ~bit;

    /* Edge detection for interrupt-capable ports */
    if (p->has_interrupt && old_value != value) {
        if ((p->ies & bit) == 0) {
            /* IES=0: rising edge (low → high) */
            if (value) {
                p->ifg |= bit;
                update_interrupt(gpio, idx);
            }
        } else {
            /* IES=1: falling edge (high → low) */
            if (!value) {
                p->ifg |= bit;
                update_interrupt(gpio, idx);
            }
        }
    }
}
