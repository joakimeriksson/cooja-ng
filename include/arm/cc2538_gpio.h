/*
 * CC2538 GPIO peripheral (Ports A-D)
 */
#ifndef CC2538_GPIO_H
#define CC2538_GPIO_H

#include "arm_cpu.h"

/* GPIO port base addresses */
#define GPIO_A_BASE  0x400D9000
#define GPIO_B_BASE  0x400DA000
#define GPIO_C_BASE  0x400DB000
#define GPIO_D_BASE  0x400DC000

/* GPIO register offsets */
#define GPIO_DATA       0x000   /* Data register (data-aliased: 0x000-0x3FC) */
#define GPIO_DIR        0x400   /* Direction register */
#define GPIO_IS         0x404   /* Interrupt Sense */
#define GPIO_IBE        0x408   /* Interrupt Both Edges */
#define GPIO_IEV        0x40C   /* Interrupt Event */
#define GPIO_IE         0x410   /* Interrupt Mask */
#define GPIO_RIS        0x414   /* Raw Interrupt Status */
#define GPIO_MIS        0x418   /* Masked Interrupt Status */
#define GPIO_ICR        0x41C   /* Interrupt Clear */
#define GPIO_AFSEL      0x420   /* Alternate Function Select */
#define GPIO_GPIOLOCK   0x520   /* GPIO Lock */
#define GPIO_GPIOCR     0x524   /* GPIO Commit */
#define GPIO_PMUX       0x700   /* PMUX register */

/* Number of GPIO ports */
#define CC2538_GPIO_NUM_PORTS 4

typedef struct cc2538_gpio_port {
    uint32_t data;      /* Output data */
    uint32_t dir;       /* Direction (1=output) */
    uint32_t is;
    uint32_t ibe;
    uint32_t iev;
    uint32_t ie;
    uint32_t ris;
    uint32_t afsel;
    uint32_t gpiolock;
    uint32_t gpiocr;
    uint32_t pmux;
} cc2538_gpio_port_t;

typedef void (*cc2538_gpio_output_fn)(void *user_data, int port, uint8_t old_val, uint8_t new_val);

typedef struct cc2538_gpio {
    arm_cpu_t             *cpu;
    cc2538_gpio_port_t     ports[CC2538_GPIO_NUM_PORTS];
    int                    irq_nums[CC2538_GPIO_NUM_PORTS];

    cc2538_gpio_output_fn  output_callback;
    void                  *output_user_data;
} cc2538_gpio_t;

/* Initialize GPIO and register IO regions */
void cc2538_gpio_init(cc2538_gpio_t *gpio, arm_cpu_t *cpu);

/* Set output change callback */
void cc2538_gpio_set_output_callback(cc2538_gpio_t *gpio,
                                     cc2538_gpio_output_fn cb, void *user_data);

/* Set input pin value (for external devices driving GPIO) */
void cc2538_gpio_set_input(cc2538_gpio_t *gpio, int port, int pin, bool value);

#endif /* CC2538_GPIO_H */
