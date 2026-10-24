/**
 * key.c - STM8S Key Driver Implementation (Object-Oriented)
 *
 * Hardware notes (STM8S103F3P6):
 *   - Button: PA3 to GND, internal pull-up enabled
 *   - EXTI:   PA3 belongs to Port A -> EXTI0 interrupt
 *             (vector #5 in SDCC, NOT EXTI2)
 *   - Trigger: falling edge only (EXTI_CR1 bit1:0 = 10)
 *   - CR2 bit of the pin must be set to enable the EXTI line
 *
 * The ISR only sets a flag; debounce and the actual action
 * are handled by key_onTime() driven by the TIM1 1ms ISR.
 *
 * BUGFIX (2026-10-24): key_exti_service must NOT read the pin level to
 * decide whether a press happened. EXTI is a falling-edge interrupt; this
 * ISR can be queued behind the TIM1 1ms ISR (same priority, no nesting on
 * STM8S by default). A quick tap's pin can bounce back high before this
 * ISR finally runs, so a level check would miss the press entirely —
 * observed as "single/double click never fires, only long press works".
 * Fix: latch the press event for every key registered on this port
 * unconditionally; key_onTime()'s DEBOUNCE state filters false triggers
 * by requiring DEBOUNCE_MS of continuous low.
 */

#include <stdint.h>
#include "key.h"

volatile uint8_t key_pressed_mask = 0;

/* Constructor bit allocator: 1 bit per KEY object, up to 8 keys.
 * Each EXTI ISR writes its own object's bit; onTime() consumes its own. */
static uint8_t next_key_bit = 0x01;

/* ========== Per-Port EXTI Registration ==========
 * STM8S103 EXTI is grouped by port: PA->EXTI0, PB->EXTI1, PC->EXTI2,
 * PD->EXTI3 (separate vectors AND separate EXTI_CR1 sensitivity bits).
 * Each KEY object registers into its port's service list at init();
 * the four EXTI ISR stubs in the main unit just call key_exti_service(group).
 * Adding a key on ANY port/pin therefore needs no ISR code changes. */
#define KEY_MAX_PER_PORT  4
static KEY_t *port_keys[4][KEY_MAX_PER_PORT];
static uint8_t port_key_cnt[4];

/* EXTI_CR1 sensitivity per group: 2 bits each, 10b = falling edge only.
 *   bit1:0 = PA(EXTI0), bit3:2 = PB(EXTI1), bit5:4 = PC(EXTI2), bit7:6 = PD(EXTI3) */
static const uint8_t exti_cr1_falling[4] = { 0x02, 0x08, 0x20, 0x80 };
static const uint8_t exti_cr1_mask[4]    = { 0x03, 0x0C, 0x30, 0xC0 };

/* Map GPIO port base to EXTI group (STM8S GPIO bases are 5 bytes apart).
 * SDCC emits STM8S hardware divw for unsigned 16-bit /5: subw+ldw+divw+
 * ld+ret = 9 bytes, fixed cycles, vs a 4-case compare chain for switch
 * (~33 bytes, worst case 4 compares). Clamp to 0..3 so an invalid
 * port_addr cannot index past port_keys[4][]. */
static uint8_t key_port_group(uint16_t port_addr)
{
    uint8_t g = (uint8_t)((port_addr - 0x5000u) / 5u);
    return (g <= 3) ? g : 0;   /* PA -> EXTI0, PB -> EXTI1, PC -> EXTI2, PD -> EXTI3 */
}

/* ========== KEY Method Implementations ========== */

static void key_init(KEY_t *self)
{
    uint16_t port = self->port_addr;
    uint8_t  mask = self->pin_mask;
    uint8_t  g    = self->port_group;

    GPIO_DDR(port) &= (uint8_t)~mask;   /* direction = input         */
    GPIO_CR1(port) |=  mask;            /* pull-up input             */
    GPIO_CR2(port) |=  mask;            /* enable EXTI line (mandatory on STM8S) */

    /* Per-port sensitivity: keep other groups untouched, set this group to
     * falling edge only (10b). 10b preserved for other groups (EXTI_CR1
     * resets to 0b00: low level, so a level-triggered EXTI could fire forever). */
    EXTI_CR1 = (EXTI_CR1 & (uint8_t)~exti_cr1_mask[g]) | exti_cr1_falling[g];

    /* Register self into this port's EXTI service list (idempotent) */
    uint8_t i;
    for (i = 0; i < port_key_cnt[g]; i++) {
        if (port_keys[g][i] == self) return;   /* already registered */
    }
    if (port_key_cnt[g] < KEY_MAX_PER_PORT) {
        port_keys[g][port_key_cnt[g]++] = self;
    }
}

static uint8_t key_read(KEY_t *self)
{
    /* raw pin level: 1 if high, 0 if low */
    return (GPIO_IDR(self->port_addr) & self->pin_mask) ? 1 : 0;
}

static uint8_t key_isPressed(KEY_t *self)
{
    /* active low: pressed when the pin reads 0 */
    return key_read(self) ? 0 : 1;
}

/* ========== Non-Blocking Click / Double-Click / Long-Press FSM ========== */

/* FSM states:
 *   IDLE         - waiting for falling edge (pressed_bit)
 *   DEBOUNCE     - confirm press: pin stable low for DEBOUNCE_MS
 *                  (seq 0 = first press, seq 1 = 2nd press of double)
 *   PRESSED      - first press confirmed; waiting release / long-press
 *   DBL_WAIT     - released after first press; waiting 2nd press within window
 *   DBL_PRESSED  - 2nd press confirmed; waiting release / long-press
 *   LONG_PRESSED - long-press fired; waiting release */
enum { KEY_STATE_IDLE = 0, KEY_STATE_DEBOUNCE, KEY_STATE_PRESSED,
       KEY_STATE_DBL_WAIT, KEY_STATE_DBL_PRESSED, KEY_STATE_LONG_PRESSED };

#define DEBOUNCE_MS        20    /* stable-low time to confirm a press */
#define RELEASE_DEBOUNCE_MS 30   /* stable-high time to confirm a release */
#define DBL_CLICK_MS       300   /* max gap between the two clicks */
#define LONG_PRESS_MS      1000  /* hold time to qualify as long-press */

/* Unsigned 16-bit tick difference: safe across uint16 wrap-around */
#define TICK_DIFF(now, prev) ((uint16_t)((now) - (prev)))

static void key_onTime(KEY_t *self, uint16_t tick)
{
    /* pressed_bit is ONLY a pending edge notification for IDLE / DBL_WAIT.
     * Every other state polls the pin directly, so clear the bit each tick
     * there: EXTI bounces during debounce may re-set it, and a stale bit
     * must never be mistaken for a brand-new press. */
    if (self->state != KEY_STATE_IDLE && self->state != KEY_STATE_DBL_WAIT) {
        key_pressed_mask &= (uint8_t)~self->pressed_bit;
    }

    switch (self->state) {

    case KEY_STATE_IDLE:
        if (key_pressed_mask & self->pressed_bit) { /* falling edge occurred */
            key_pressed_mask &= (uint8_t)~self->pressed_bit; /* consume now */
            self->debounce_start = tick;
            self->seq = 0;                          /* first press */
            self->state = KEY_STATE_DEBOUNCE;
        }
        break;

    case KEY_STATE_DEBOUNCE:
        if (key_isPressed(self)) {                  /* still low: keep timing */
            if (TICK_DIFF(tick, self->debounce_start) >= DEBOUNCE_MS) {
                self->press_start = tick;           /* press confirmed */
                self->debounce_start = tick;        /* reuse as release timer */
                self->state = (self->seq == 0) ? KEY_STATE_PRESSED
                                               : KEY_STATE_DBL_PRESSED;
            }
        } else {                                    /* bounced high: restart */
            self->debounce_start = tick;
            self->state = (self->seq == 0) ? KEY_STATE_IDLE
                                           : KEY_STATE_DBL_WAIT;
            /* 2nd-press attempt bounced: back to DBL_WAIT, window still open */
        }
        break;

    case KEY_STATE_PRESSED:                         /* first press confirmed */
        if (key_isPressed(self)) {
            self->debounce_start = tick;            /* reset release timer */
            if (TICK_DIFF(tick, self->press_start) >= LONG_PRESS_MS) {
                if (self->onlongpress) self->onlongpress(self);
                self->state = KEY_STATE_LONG_PRESSED; /* long press fired */
            }
        } else if (TICK_DIFF(tick, self->debounce_start)
                   >= RELEASE_DEBOUNCE_MS) {        /* release confirmed */
            self->release_start = tick;
            self->state = KEY_STATE_DBL_WAIT;       /* look for 2nd press */
        }
        break;

    case KEY_STATE_DBL_WAIT:
        if (key_pressed_mask & self->pressed_bit) { /* 2nd press inside window */
            key_pressed_mask &= (uint8_t)~self->pressed_bit; /* consume */
            self->debounce_start = tick;
            self->seq = 1;                          /* second press */
            self->state = KEY_STATE_DEBOUNCE;
        } else if (TICK_DIFF(tick, self->release_start) >= DBL_CLICK_MS) {
            if (self->onclick) self->onclick(self); /* window closed: single */
            self->state = KEY_STATE_IDLE;
        }
        break;

    case KEY_STATE_DBL_PRESSED:                     /* 2nd press confirmed */
        if (key_isPressed(self)) {
            self->debounce_start = tick;
            if (TICK_DIFF(tick, self->press_start) >= LONG_PRESS_MS) {
                if (self->onlongpress) self->onlongpress(self);
                self->state = KEY_STATE_LONG_PRESSED;
            }
        } else if (TICK_DIFF(tick, self->debounce_start)
                   >= RELEASE_DEBOUNCE_MS) {
            if (self->ondblclick) self->ondblclick(self); /* double click fired */
            self->state = KEY_STATE_IDLE;
        }
        break;

    case KEY_STATE_LONG_PRESSED:                    /* wait release after long */
        if (key_isPressed(self)) {
            self->debounce_start = tick;
        } else if (TICK_DIFF(tick, self->debounce_start)
                   >= RELEASE_DEBOUNCE_MS) {
            self->state = KEY_STATE_IDLE;
        }
        break;
    }
}

/* ========== KEY Constructor ========== */

KEY_t KEY_Create(uint16_t port_addr, uint8_t pin)
{
    KEY_t key;

    key.port_addr   = port_addr;
    key.pin         = pin;
    key.pin_mask    = (uint8_t)(1 << pin);
    key.active_level = 0;                /* active low (button to GND) */
    key.port_group  = key_port_group(port_addr);   /* PA/PB/PC/PD -> EXTI0..3 */

    /* Bind methods */
    key.init        = key_init;
    key.read        = key_read;
    key.isPressed   = key_isPressed;
    key.onTime      = key_onTime;
    key.onclick     = 0;              /* default: no callback, assign in app */
    key.ondblclick  = 0;
    key.onlongpress = 0;

    /* Init internal FSM state */
    key.state          = KEY_STATE_IDLE;
    key.debounce_start = 0;
    key.press_start    = 0;
    key.release_start  = 0;
    key.seq            = 0;

    /* Allocate own ISR flag bit (up to 8 KEY objects; wraps after bit7) */
    key.pressed_bit    = next_key_bit;
    next_key_bit <<= 1;
    if (next_key_bit == 0x00) {
        next_key_bit = 0x01;
    }

    return key;
}

/* ========== EXTI Port Service ==========
 * Called from the four EXTI ISR stubs in the main unit:
 *   EXTI0_IRQHandler -> key_exti_service(0)   (Port A)
 *   EXTI1_IRQHandler -> key_exti_service(1)   (Port B)
 *   EXTI2_IRQHandler -> key_exti_service(2)   (Port C)
 *   EXTI3_IRQHandler -> key_exti_service(3)   (Port D)
 * Latches a press for every KEY object registered on that port (see the
 * BUGFIX note above for why level reading was removed). New keys on any
 * port/pin need zero ISR changes. */
void key_exti_service(uint8_t group)
{
    /* EXTI fired on this port = at least one registered pin saw a falling
     * edge (EXTI_CR1 is configured falling-edge only).
     *
     * Latch the press for ALL keys on this port — do NOT read the pin
     * level here. This ISR can be delayed behind the TIM1 1ms ISR (same
     * priority, no nesting); a quick tap's pin may already have bounced
     * back high when we finally read IDR, and a level check would miss
     * the press entirely. Unconditional latching + the FSM's DEBOUNCE
     * state (DEBOUNCE_MS of continuous low) is immune to that race.
     * Keys that did NOT actually move leave DEBOUNCE on the next tick
     * (pin reads high) and go back to their previous state — harmless. */
    uint8_t i;
    for (i = 0; i < port_key_cnt[group]; i++) {
        key_pressed_mask |= port_keys[group][i]->pressed_bit;
    }
}

/* ========== EXTI 中断 ==========
 * 注意：SDCC stm8 仅在包含 main() 的编译单元生成 __interrupt_vect 向量段，
 * 本文件不含 main()，在此定义 __interrupt ISR 不会生成向量条目（函数体
 * 存在但向量表为 0x000000，中断触发将导致复位）。
 * EXTI0 ISR 已移至 led_blink_main.c（main 所在单元）定义，见其末尾。
 */
