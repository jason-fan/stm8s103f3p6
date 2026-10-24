/**
 * led.h - STM8S LED Driver (Object-Oriented)
 * 
 * Provides GPIO register access macros and a polymorphic LED_t
 * abstraction with method binding. Supports any GPIO port and pin.
 * 
 * Usage:
 *   #include "led.h"
 *   LED_t led = LED_Create(0x5000, 3);  // PA3
 *   led.init(&led);
 *   led.toggle(&led);
 */

#ifndef LED_H
#define LED_H

#include <stdint.h>

/* ========== GPIO Register Offset Macros ==========
 * STM8S GPIO registers layout per port:
 *   +0: ODR    Output Data Register
 *   +1: IDR    Input Data Register
 *   +2: DDR    Data Direction Register
 *   +3: CR1    Control Register 1
 *   +4: CR2    Control Register 2
 */

#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_ODR(port)          GPIO_REG(port, 0)
#define GPIO_IDR(port)          GPIO_REG(port, 1)
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* ========== LED Class Definition ========== */

typedef struct LED LED_t;

struct LED {
    uint16_t port_addr;     /* GPIO port base address */
    uint8_t  pin;           /* pin number (0-7) */
    uint8_t  pin_mask;      /* precomputed bit mask (1 << pin) */
    uint8_t  active_high;   /* 1 = ODR high lights LED, 0 = ODR low lights LED */

    /* --- Public methods --- */
    void (*init)  (LED_t *self);
    void (*on)    (LED_t *self);
    void (*off)   (LED_t *self);
    void (*toggle)(LED_t *self);
    void (*onTime)(LED_t *self, uint16_t tick);   /* 1ms periodic event, called from TIM1 ISR */

    /* --- Blink timing (asymmetric: on_ms ON / off_ms OFF) --- */
    uint16_t on_ms;          /* duration LED stays ON  (default 500ms)   */
    uint16_t off_ms;         /* duration LED stays OFF (default 1000ms)  */
    uint8_t  phase;          /* 1 = ON phase, 0 = OFF phase */

    /* --- Internal blink state (managed by led.c, do not touch) --- */
    uint16_t last_tick;      /* tick when current phase started */
};

/* ========== Constructor ========== */

/**
 * Create an LED object and bind to specified port/pin.
 * Uses static storage — safe for single-instance use on STM8S103.
 * Initial phase is ON: power-on lights the LED for on_ms, then off_ms.
 *
 * @param port_addr   GPIO port base address (e.g. 0x5000 for PA)
 * @param pin         GPIO pin number (0-7)
 * @param active_high 1 = LED lights when pin is high (anode to pin),
 *                    0 = LED lights when pin is low (cathode to pin)
 * @return            fully initialized LED_t object
 */
LED_t LED_Create(uint16_t port_addr, uint8_t pin, uint8_t active_high);

#endif /* LED_H */
