/**
 * key.h - STM8S Key Driver (Object-Oriented, PA3 Button + EXTI)
 *
 * Provides GPIO register access macros and a polymorphic KEY_t
 * abstraction with method binding, mirroring LED_t in led.h.
 * Supports any GPIO port and pin on STM8S103.
 *
 * Usage:
 *   #include "key.h"
 *   KEY_t key = KEY_Create(0x5000, 3);   // PA3
 *   key.init(&key);                      // input + pull-up + EXTI
 *   __asm__("rim");                      // enable global interrupts
 *   ... in main loop ...
 *   if (key_pressed_mask & key.pressed_bit) { ... key.isPressed(&key) ... }
 */

#ifndef KEY_H
#define KEY_H

#include <stdint.h>

/* ========== GPIO Register Offset Macros ==========
 * STM8S GPIO registers layout per port:
 *   +0: ODR    Output Data Register
 *   +1: IDR    Input Data Register
 *   +2: DDR    Data Direction Register
 *   +3: CR1    Control Register 1
 *   +4: CR2    Control Register 2
 * (identical definitions to led.h — safe to co-include)
 */

#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_ODR(port)          GPIO_REG(port, 0)
#define GPIO_IDR(port)          GPIO_REG(port, 1)
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* ========== STM8S103 Register Addresses ========== */

#define EXTI_CR1    (*(volatile uint8_t *)0x50A0)   /* EXTI sensitivity */

/* ========== Key Configuration (PA3) ========== */

#define KEY_PIN         3
#define KEY_PIN_MASK    (uint8_t)(1 << KEY_PIN)     /* 0x08 */

/* ========== Key Class Definition ========== */

typedef struct KEY KEY_t;

struct KEY {
    uint16_t port_addr;     /* GPIO port base address */
    uint8_t  pin;           /* pin number (0-7) */
    uint8_t  pin_mask;      /* precomputed bit mask (1 << pin) */
    uint8_t  active_level;  /* 0 = active low (button to GND) */

    /* --- Public methods --- */
    void    (*init)      (KEY_t *self);              /* input + pull-up + EXTI */
    uint8_t (*read)      (KEY_t *self);              /* raw pin level: 1/0 */
    uint8_t (*isPressed) (KEY_t *self);              /* pressed state (active low) */
    void    (*onTime)    (KEY_t *self, uint16_t tick); /* 1ms periodic event: non-blocking FSM (TIM1 ISR) */
    void    (*onclick)   (KEY_t *self);              /* single-click callback (nullable) */
    void    (*ondblclick)(KEY_t *self);              /* double-click callback (nullable) */
    void    (*onlongpress)(KEY_t *self);             /* long-press callback (nullable) */

    /* --- Internal FSM state (managed by key.c, do not touch) --- */
    uint8_t  state;            /* KEY_STATE_* */
    uint16_t debounce_start;   /* tick when debounce began */
    uint16_t press_start;      /* tick when press confirmed */
    uint16_t release_start;    /* tick when release began */
    uint8_t  seq;              /* 0 = first press, 1 = second press of double */

    /* --- ISR ownership (managed by key.c constructor) --- */
    uint8_t  port_group;       /* EXTI group: 0=PA,1=PB,2=PC,3=PD (from port_addr) */
    uint8_t  pressed_bit;      /* bit mask in key_pressed_mask, 1 bit per KEY object */
};

/* ========== Public State ========== */

/* Falling-edge flag, one bit per KEY object (allocated by KEY_Create).
 * The per-port EXTI ISR (EXTI0..EXTI3) calls key_exti_service(group),
 * which scans that port's registered KEY objects, reads GPIO IDR to find
 * which pin(s) actually went low, and sets only those objects' bits.
 * key.onTime() consumes and clears only self->pressed_bit.
 * Adding a new key on any pin/port needs NO ISR changes. */
extern volatile uint8_t key_pressed_mask;

/* Per-port EXTI service: scan group's registered keys, set pressed bits
 * for pins reading low (falling edge). Called by EXTI0..EXTI3 ISR stubs
 * in the main unit (SDCC only emits the vector table there). */
void key_exti_service(uint8_t group);

/* ========== Constructor ========== */

/**
 * Create a KEY object and bind to specified port/pin.
 * Uses static storage — safe for single-instance use on STM8S103.
 * The constructor does NOT configure the pin; call init() first.
 *
 * @param port_addr  GPIO port base address (e.g. 0x5000 for PA)
 * @param pin        GPIO pin number (0-7)
 * @return           fully initialized KEY_t object
 */
KEY_t KEY_Create(uint16_t port_addr, uint8_t pin);

#endif /* KEY_H */
