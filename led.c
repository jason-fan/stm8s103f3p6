/**
 * led.c - STM8S LED Driver Implementation
 */

#include "led.h"

/* ========== LED Method Implementations ========== */

/* Set LED brightness: level 1 = lit, 0 = unlit.
 * Polarity handled here: active_high → ODR high = lit,
 *                         active_low  → ODR low  = lit.  */
static void led_set_level(LED_t *self, uint8_t level)
{
    uint8_t odr_val = self->active_high ? level : (uint8_t)(level ^ 1);

    if (odr_val)
        GPIO_ODR(self->port_addr) |= self->pin_mask;
    else
        GPIO_ODR(self->port_addr) &= ~self->pin_mask;
}

static void led_init(LED_t *self)
{
    uint16_t port = self->port_addr;
    uint8_t  mask = self->pin_mask;

    GPIO_DDR(port) |=  mask;   /* direction = output       */
    GPIO_CR1(port) |=  mask;   /* push-pull mode           */
    GPIO_CR2(port) &= ~mask;   /* 2MHz slew rate (default) */
    led_set_level(self, 1);    /* power-on: LED lit (ON phase) */
}

static void led_on(LED_t *self)
{
    led_set_level(self, 1);
}

static void led_off(LED_t *self)
{
    led_set_level(self, 0);
}

static void led_toggle(LED_t *self)
{
    GPIO_ODR(self->port_addr) ^= self->pin_mask;
}

static void led_onTime(LED_t *self, uint16_t tick)
{
    /* Asymmetric blink: ON for on_ms, OFF for off_ms (500ms / 1000ms).
     * Unsigned diff handles uint16 wrap-around; ISR-safe: no blocking. */
    uint16_t elapsed = (uint16_t)(tick - self->last_tick);

    if (self->phase) {                    /* ON phase: count down to off */
        if (elapsed >= self->on_ms) {
            self->phase     = 0;
            self->last_tick = tick;
            led_off(self);
        }
    } else {                              /* OFF phase: count down to on  */
        if (elapsed >= self->off_ms) {
            self->phase     = 1;
            self->last_tick = tick;
            led_on(self);
        }
    }
}

/* ========== LED Constructor ========== */

LED_t LED_Create(uint16_t port_addr, uint8_t pin, uint8_t active_high)
{
    LED_t led;

    led.port_addr   = port_addr;
    led.pin         = pin;
    led.pin_mask    = (uint8_t)(1 << pin);
    led.active_high = active_high;
    led.last_tick   = 0;
    led.phase       = 1;   /* start in ON phase (init lights LED) */
    led.on_ms       = 500; /* LED on  500ms  */
    led.off_ms      = 1000;/* LED off 1000ms */

    /* Bind methods */
    led.init   = led_init;
    led.on     = led_on;
    led.off    = led_off;
    led.toggle = led_toggle;
    led.onTime = led_onTime;

    return led;
}
