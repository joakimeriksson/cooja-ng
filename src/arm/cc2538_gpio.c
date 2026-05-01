/*
 * CC2538 GPIO peripheral (Ports A-D)
 */
#include "cc2538_gpio.h"
#include "arm_nvic.h"
#include <string.h>

#define GPIO_PORT_SIZE  0x1000

static const uint32_t gpio_bases[CC2538_GPIO_NUM_PORTS] = {
    GPIO_A_BASE, GPIO_B_BASE, GPIO_C_BASE, GPIO_D_BASE
};

static int gpio_read(void *user_data, uint32_t addr) {
    cc2538_gpio_t *gpio = (cc2538_gpio_t *)user_data;

    for (int p = 0; p < CC2538_GPIO_NUM_PORTS; p++) {
        if (addr >= gpio_bases[p] && addr < gpio_bases[p] + GPIO_PORT_SIZE) {
            uint32_t offset = addr - gpio_bases[p];
            cc2538_gpio_port_t *port = &gpio->ports[p];

            if (offset < 0x400) {
                /* Data-aliased register: bits [9:2] of address select which bits */
                uint8_t mask = (offset >> 2) & 0xFF;
                return (int)(port->data & mask);
            }

            switch (offset) {
                case GPIO_DIR:     return (int)port->dir;
                case GPIO_IS:      return (int)port->is;
                case GPIO_IBE:     return (int)port->ibe;
                case GPIO_IEV:     return (int)port->iev;
                case GPIO_IE:      return (int)port->ie;
                case GPIO_RIS:     return (int)port->ris;
                case GPIO_MIS:     return (int)(port->ris & port->ie);
                case GPIO_ICR:     return 0;
                case GPIO_AFSEL:   return (int)port->afsel;
                case GPIO_GPIOLOCK: return (int)port->gpiolock;
                case GPIO_GPIOCR:  return (int)port->gpiocr;
                case GPIO_PMUX:    return (int)port->pmux;
                default:           return 0;
            }
        }
    }
    return 0;
}

static void gpio_write(void *user_data, uint32_t addr, uint32_t value) {
    cc2538_gpio_t *gpio = (cc2538_gpio_t *)user_data;

    for (int p = 0; p < CC2538_GPIO_NUM_PORTS; p++) {
        if (addr >= gpio_bases[p] && addr < gpio_bases[p] + GPIO_PORT_SIZE) {
            uint32_t offset = addr - gpio_bases[p];
            cc2538_gpio_port_t *port = &gpio->ports[p];

            if (offset < 0x400) {
                /* Data-aliased write */
                uint8_t mask = (offset >> 2) & 0xFF;
                uint8_t old_data = port->data;
                port->data = (port->data & ~mask) | (value & mask);
                if (gpio->output_callback && old_data != port->data) {
                    gpio->output_callback(gpio->output_user_data, p,
                                         old_data, port->data);
                }
                return;
            }

            switch (offset) {
                case GPIO_DIR:     port->dir = value & 0xFF; break;
                case GPIO_IS:      port->is = value & 0xFF; break;
                case GPIO_IBE:     port->ibe = value & 0xFF; break;
                case GPIO_IEV:     port->iev = value & 0xFF; break;
                case GPIO_IE:      port->ie = value & 0xFF; break;
                case GPIO_ICR:     port->ris &= ~(value & 0xFF); break;
                case GPIO_AFSEL:   port->afsel = value & 0xFF; break;
                case GPIO_GPIOLOCK: port->gpiolock = value; break;
                case GPIO_GPIOCR:  port->gpiocr = value & 0xFF; break;
                case GPIO_PMUX:    port->pmux = value; break;
            }
            return;
        }
    }
}

void cc2538_gpio_init(cc2538_gpio_t *gpio, arm_cpu_t *cpu) {
    memset(gpio, 0, sizeof(*gpio));
    gpio->cpu = cpu;

    /* IRQ numbers for GPIO ports A-D */
    gpio->irq_nums[0] = 0;  /* GPIO A */
    gpio->irq_nums[1] = 1;  /* GPIO B */
    gpio->irq_nums[2] = 2;  /* GPIO C */
    gpio->irq_nums[3] = 3;  /* GPIO D */

    for (int p = 0; p < CC2538_GPIO_NUM_PORTS; p++) {
        arm_register_io(cpu, gpio_bases[p], GPIO_PORT_SIZE,
                        gpio_read, gpio_write, gpio);
    }
}

void cc2538_gpio_set_output_callback(cc2538_gpio_t *gpio,
                                     cc2538_gpio_output_fn cb, void *user_data) {
    gpio->output_callback = cb;
    gpio->output_user_data = user_data;
}

void cc2538_gpio_set_input(cc2538_gpio_t *gpio, int port, int pin, bool value) {
    if (port < 0 || port >= CC2538_GPIO_NUM_PORTS) return;
    if (pin < 0 || pin > 7) return;

    cc2538_gpio_port_t *p = &gpio->ports[port];
    uint8_t mask = (1 << pin);

    if (!(p->dir & mask)) { /* Only set if pin is input */
        if (value)
            p->data |= mask;
        else
            p->data &= ~mask;
    }
}

void cc2538_gpio_force_irq_edge(cc2538_gpio_t *gpio, int port, int pin, bool rising) {
    if (port < 0 || port >= CC2538_GPIO_NUM_PORTS) return;
    if (pin < 0 || pin > 7) return;

    cc2538_gpio_port_t *p = &gpio->ports[port];
    uint8_t mask = (uint8_t)(1u << pin);

    p->ie |= mask;
    if (rising) p->ris |=  mask;
    else        p->ris &= ~mask;

    /* Pend the port's NVIC IRQ if any RIS bit is now masked-in. */
    if ((p->ris & p->ie) && gpio->cpu && gpio->cpu->nvic && gpio->irq_nums[port] >= 0) {
        arm_nvic_set_pending((arm_nvic_t *)gpio->cpu->nvic, gpio->irq_nums[port]);
    }
}
