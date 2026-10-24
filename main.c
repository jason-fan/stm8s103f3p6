/**
 * STM8S103F3P6 LED Blink Example (Object-Oriented Style)
 * 
 * Target MCU:   STM8S103F3P6
 * Compiler:     SDCC (Small Device C Compiler)
 * LED Pin:      PA3 (Push-pull output, active high)
 * Clock:        HSI 16MHz (default after reset)
 * Blink rate:   1.5s toggle via LED.onTime (TIM1 1ms ISR time-slice)
 * Event loop:   TIM1 hard interrupt schedules led.onTime / key.onTime
 */

#include <stdint.h>
#include "led.h"
#include "key.h"
#include "uart.h"
#include "eeprom.h"
#include "adc.h"
#include "beep.h"

/*
PD4 beep蜂鸣器
PD5 TXD <--> TTL:rxd
PD6 RXD <--> TTL:txd
GND      按键 电位器 蜂鸣器 UART
3.3v --> 电位器上
PA3  --> 按键
PC4  --> 电位器中心抽头,AD转换
*/

/* ========== User Configuration ========== */

#define GPIO_PA         0x5000
#define GPIO_PB         0x5005
#define GPIO_PC         0x500A
#define GPIO_PD         0x500F

#define LED_PORT_ADDR   GPIO_PB   /* GPIO port base */
#define LED_PIN         5         /* GPIO pin number (0-7) PB5 */
#define LED_ACTIVE_HIGH 0         /* 0 = cathode to PB5, LED lights when pin is low */

#define KEY_PORT_ADDR   GPIO_PA   /* GPIO port base */
#define KEY_PIN_NUM     3        /* GPIO pin number (0-7) PA3 */

/* GPIO PB output data register (GPIO_PB base = ODR register on STM8) */
#define GPIO_PB_ODR     (*(volatile uint8_t *)GPIO_PB)

/* ========== STM8S103 Register Definitions ========== */

/* Clock registers */
#define CLK_CKDIVR  (*(volatile uint8_t *)0x50C6)

/* ========== Clock Configuration ========== */

static void clock_setup(void)
{
    CLK_CKDIVR = 0x00;  /* HSI 16MHz, no division */
}

/* ========== Global Objects (file-level, accessible from ISR) ========== */

static LED_t led;                  /* file-level LED object (ISR + callback) */
static KEY_t key;                  /* file-level KEY object PA3 (ISR) */
static UART_t uart;                /* file-level UART object (main loop) */
static Beep_t beep;                /* file-level BEEP object PD4 (main loop) */

/* Global 1ms tick, incremented by TIM1 ISR (volatile for ISR/main share) */
volatile uint16_t g_tick = 0;

/* ========== Key Event Callbacks ==========
 * UART output now goes through the TX ring buffer: uart.printf only
 * ENQUEUES bytes and returns immediately, so these callbacks are safe
 * to run inside the TIM1 ISR. The main loop drains the buffer with
 * uart.tx_service(). No global event counter/type variables needed.
 *
 * Every event prints the triggering key's pin position (e.g.
 * "KEY SINGLE PA3") from self->port_addr / self->pin, so the same
 * callback shape serves any key on any port. */

static char key_port_char(uint16_t port_addr)
{
    switch (port_addr) {
    case GPIO_PA: return 'A';
    case GPIO_PB: return 'B';
    case GPIO_PC: return 'C';
    case GPIO_PD: return 'D';
    default:      return '?';
    }
}

static void led_toggle_onclick(KEY_t *self)
{
    led.toggle(&led);              /* PA3 single click: toggle LED */
    beep.once(&beep, 4, 120);       /* single click: 4kHz short beep */
    uart.printf(&uart, "KEY SINGLE %c%u\r\n",
                (int)key_port_char(self->port_addr),
                (unsigned int)self->pin);
}

static void key_ondblclick(KEY_t *self)
{
    beep.once(&beep, 2, 100);      /* double click: 2kHz beep */
    uart.printf(&uart, "KEY DOUBLE %c%u\r\n",
                (int)key_port_char(self->port_addr),
                (unsigned int)self->pin);
}

static void key_onlongpress(KEY_t *self)
{
    beep.once(&beep, 1, 150);      /* long press: 1kHz beep */
    uart.printf(&uart, "KEY LONG %c%u\r\n",
                (int)key_port_char(self->port_addr),
                (unsigned int)self->pin);
}

/* ========== TIM1 1ms Time-Slice Timer (STM8S103) ========== */

/* STM8S103F3 官方寄存器映射（datasheet Table 8，基址 0x5250）：
 *   CR1=0x5250  IER=0x5254  SR1=0x5255  EGR=0x5257
 *   PSCRH=0x5260  PSCRL=0x5261  ARRH=0x5262  ARRL=0x5263 */
#define TIM1_CR1    (*(volatile uint8_t *)0x5250)
#define TIM1_IER    (*(volatile uint8_t *)0x5254)
#define TIM1_SR1    (*(volatile uint8_t *)0x5255)
#define TIM1_EGR    (*(volatile uint8_t *)0x5257)
#define TIM1_PSCRH  (*(volatile uint8_t *)0x5260)
#define TIM1_PSCRL  (*(volatile uint8_t *)0x5261)
#define TIM1_ARRH   (*(volatile uint8_t *)0x5262)
#define TIM1_ARRL   (*(volatile uint8_t *)0x5263)

/* HSI 16MHz, prescaler /16 -> 1MHz counter, ARR=1000 -> 1ms update */
static void tim1_setup(void)
{
    TIM1_PSCRH = 0x00;   /* prescaler high byte (16-bit prescaler = 16) */
    TIM1_PSCRL = 15;     /* /16 */
    TIM1_ARRH  = 0x03;   /* auto-reload 1000 (0x03E8) */
    TIM1_ARRL  = 0xE8;
    TIM1_EGR  |= 0x01;   /* UG: generate update to load prescaler/ARR */
    TIM1_IER  |= 0x01;   /* UIE: enable update interrupt */
    TIM1_CR1  |= 0x01;   /* CEN: start counter */
}

/* ========== TIM1 Update ISR ==========
 * SDCC stm8 向量号映射：__interrupt(n) -> 0x8000 + (n+2)*4（本地编译反查验证）。
 * STM8S103F3 硬件 TIM1_UPD_OVF_TRG_BRK 位于 0x8034（IRQ 11），
 * 对应 SDCC __interrupt(11)。上一轮误用 __interrupt(9) 会落在
 * 0x802C（保留向量），TIM1 中断触发后跳转未定义向量，导致故障依旧。 */

void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(11)
{
    TIM1_SR1 &= (uint8_t)~0x01;      /* clear UIF (must do before exit) */

    g_tick++;                        /* advance global 1ms tick */

    /* Time-slice scheduling: non-blocking periodic events only */
    led.onTime(&led, g_tick);
    key.onTime(&key, g_tick);
}

/* ========== EXTI0..EXTI3 ISRs (per-port) ==========
 * 硬件向量：EXTI0(Port A)=0x8014(IRQ3), EXTI1(Port B)=0x8018(IRQ4),
 *           EXTI2(Port C)=0x801C(IRQ5), EXTI3(Port D)=0x8020(IRQ6).
 * SDCC 映射 __interrupt(n) -> 0x8000+(n+2)*4，故分别用 (3)(4)(5)(6)。
 * 各 ISR 只做一件事：调用 key_exti_service(group) 扫描该端口注册的
 * KEY 对象，按 IDR 置对应 pressed_bit——新增按键无论落在哪个端口/引脚
 * 都无需再改 ISR 与向量。未启用 CR2 的端口不会触发，占位无害。
 */

void EXTI0_IRQHandler(void) __interrupt(3) { key_exti_service(0); }
void EXTI1_IRQHandler(void) __interrupt(4) { key_exti_service(1); }
void EXTI2_IRQHandler(void) __interrupt(5) { key_exti_service(2); }
void EXTI3_IRQHandler(void) __interrupt(6) { key_exti_service(3); }

/* ========== UART1 RX ISR Stub (must live in the main unit!) ==========
 * SDCC 4.5.0 stm8 后端只在"含 main 的编译单元"生成 __interrupt_vect 向量表，
 * 且只收录该单元内定义的 __interrupt 函数。若 UART1_RX_IRQHandler 定义在
 * uart.c，0x8050（IRQ18）向量处不会有跳转条目（该地址被 GSINIT 启动代码
 * 占用），接收中断永远不会触发——本工程上一版实测无回显即由此导致。
 * 因此向量壳必须放在本文件（main 单元），实际处理委托给 uart 模块的
 * uart_rx_irq_handler()（UART 逻辑仍留在 uart.c，保持模块化）。
 */
void UART1_RX_IRQHandler(void) __interrupt(18)
{
    uart_rx_irq_handler();
}

/* ========== Main Entry ========== */

int main(void)
{
    uint16_t last_adc_tick = 0;      /* ADC 100ms sampling scheduling */
    uint16_t last_adc_value = 0xFFFF; /* force first ADC print */

    key = KEY_Create(KEY_PORT_ADDR, KEY_PIN_NUM);   /* PA3 */
    led = LED_Create(LED_PORT_ADDR, LED_PIN, LED_ACTIVE_HIGH);
    uart = UART_Create(UART1_BASE);

    clock_setup();                       /* HSI 16MHz */

    led.init(&led);                      /* PB5 push-pull output */
    key.init(&key);                      /* PA3 input + pull-up + EXTI */
    key.onclick      = &led_toggle_onclick;   /* PA3 single click -> toggle LED */
    key.ondblclick   = &key_ondblclick;       /* PA3 double click */
    key.onlongpress  = &key_onlongpress;      /* PA3 long press */
    uart.init(&uart);                    /* UART1 115200 8N1, TX on PD5 (debug link) */
    adc_init();                          /* ADC1 single conversion, AIN2 = PC4 */
    beep = BEEP_Create();
    beep.init(&beep);                    /* BEEP on PD4: LSI on, 1kHz off */

    tim1_setup();                        /* TIM1: 1ms update interrupt */
    __asm__("rim");                      /* enable global interrupts FIRST
                                          * (UART TXE wait has timeout now, but
                                          * keep interrupts on before any I/O) */
    uart.puts(&uart, "STM8S103 UART debug online\r\n");

    /* EEPROM boot counter: read, increment, write back, report.
     * Erased EEPROM reads 0xFFFF -> treat as first power-up (count=1).
     * uint16 wraps at 65535 and continues (acceptable for this demo). */
    {
        uint16_t boot_count = eeprom_read_u16(EEPROM_BASE_ADDR);
        if (boot_count == 0xFFFFU) {
            boot_count = 1;
        } else {
            boot_count++;
        }
        if (eeprom_write_u16(EEPROM_BASE_ADDR, boot_count)) {
            uart.printf(&uart, "Boot count=%u\r\n", (unsigned int)boot_count);
        } else {
            uart.printf(&uart, "EEPROM write FAILED (count=%u)\r\n",
                        (unsigned int)boot_count);
        }
    }

    /* Event-driven main loop with buffered UART TX.
     * Every producer (key callbacks in TIM1 ISR, RX echo, periodic
     * report) only ENQUEUES into the ring buffer via uart.printf/putc;
     * uart.tx_service() at the top of each round drains it over the
     * wire. This keeps ISR paths non-blocking and removes the old
     * global g_key_events / g_key_event_type handoff variables.
     * NOTE: 不能用 halt！官方中断映射表（datasheet Table 8）中 TIM1 的
     * "Wakeup from halt" 列为空，halt 会停掉主时钟/外设时钟，TIM1 中断
     * 无法唤醒 CPU（只有 EXTI/AWU 等可唤醒 halt）。用 wfi：CPU 暂停但
     * 外设时钟继续，任何中断（含 TIM1 1ms）唤醒后自动进入 ISR。 */
    while (1) {
        /* Drain pending TX bytes (blocks on TXE, safe in main context:
         * TIM1 ISR may preempt it and enqueue more bytes meanwhile). */
        uart.tx_service(&uart);
        beep.update(&beep);          /* ~1ms tick: auto-off timed beeps */

        /* UART RX echo: byte received via RX interrupt -> enqueue it back */
        if (uart.rx_flag) {
            uart.rx_flag = 0;
            uart.putc(&uart, (char)uart.rx_byte);
        }

        /* ADC: read PC4 (AIN2) every 100ms, convert to millivolts
         * (V_mV = raw * 3300 / 1023), print only when the value
         * changes by more than ±5mV (hysteresis for LSB jitter). */
        if ((uint16_t)(g_tick - last_adc_tick) >= 100U) {
            uint16_t adc_raw = adc_read_ch2();
            uint16_t adc_mv  = (uint16_t)(((uint32_t)adc_raw * 3300U) / 1023U);
            uint16_t delta;
            last_adc_tick = g_tick;
            delta = (adc_mv > last_adc_value)
                        ? (uint16_t)(adc_mv - last_adc_value)
                        : (uint16_t)(last_adc_value - adc_mv);
            if (delta > 5U) {
                uint16_t volt_int = (uint16_t)(adc_mv / 1000U);
                uint16_t volt_frac = (uint16_t)(adc_mv % 1000U);
                last_adc_value = adc_mv;
                /* lightweight printf has no %f / %03u -> split manually:
                 * e.g. 129mV -> "0.129V" */
                uart.printf(&uart, "pc4=%u.%u%u%uV\r\n",
                            (unsigned int)volt_int,
                            (unsigned int)(volt_frac / 100U),
                            (unsigned int)((volt_frac / 10U) % 10U),
                            (unsigned int)(volt_frac % 10U));
            }
        }

        __asm__("wfi");                /* wait-for-interrupt, woken by TIM1/EXTI */
    }

    // return 0;
}
