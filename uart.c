/**
 * uart.c - STM8S UART1 Driver Implementation (Object-Oriented)
 *
 * Hardware notes (STM8S103F3P6):
 *   - UART1 TX pin: PD5 (default mapping, AFR7=0 on this chip;
 *     verified by TX-probe firmware: PD5 idle 3.3V with TX waveform,
 *     PC3 stayed 0V)
 *   - Baud rate: UARTDIV = fck / (16 * baud), fck = HSI 16MHz
 *     115200 -> UARTDIV = 8.68:  BRR1=0x08, BRR2=0x0B
 *     (legacy 9600 -> UARTDIV = 104.17: BRR1=0x68, BRR2=0x03)
 *     STM8 BRR split: BRR1 = mantissa [7:0],
 *                     BRR2 = mantissa [11:8] << 4 | fraction [3:0]
 *   - Write order: BRR2 first, then BRR1 (BRR1 write latches the divider)
 *
 * All send functions are blocking on TXE (safe from main context;
 * TIM1 ISR may preempt the wait loop without losing ticks).
 */

#include <stdint.h>
#include <stdarg.h>
#include "uart.h"

/* ========== RX Owner (for the RX ISR) ==========
 * Set by uart_init() to the live UART_t instance; used by
 * UART1_RX_IRQHandler to publish the received byte. */
static UART_t *s_rx_owner = 0;

/* ========== UART Method Implementations ========== */

static void uart_init(UART_t *self)
{
    (void)self;                 /* single UART1 instance on STM8S103 */

    /* Force full-speed HSI: on this board the chip powers up with the
     * HSI divided down (CLK_CKDIVR not 0x00, effective fck ~2MHz), so a
     * 9600-config BRR actually transmits at 1200 baud. Setting
     * CLK_CKDIVR = 0x00 restores HSI/1 = 16MHz for both CPU and UART.
     * (The ST official library test code also set CLK_DIVR=0x00 first.) */
    CLK_CKDIVR = 0x00;

    /* Enable all peripheral clocks (same as ST official library init:
     * CLK_PCKENR1 = 0xFF). Must happen BEFORE touching UART registers,
     * otherwise writes are ignored (TEN never takes effect, TXE stays
     * unset -> putc times out, no output). */
    CLK_PCKENR1 = 0xFF;

    /* PD5 = UART1_TX (default mapping, AFR7=0 on this chip): configure
     * GPIO as push-pull output. STM8S requires DDR=1 for alternate-function
     * outputs to reach the pin. ODR is NOT set (stays 0): the UART
     * peripheral drives the line level (idle high) itself; setting ODR=1
     * can latch the pin high and block the TX waveform. */
    GPIO_PD_DDR |= UART1_TX_PIN;
    GPIO_PD_CR1 |= UART1_TX_PIN;    /* push-pull output */
    // GPIO_PD_CR2 &= (uint8_t)~UART1_TX_PIN;  /* 2MHz slew, fine for 115200 */

    UART1_CR2 = 0x00;           /* disable TX/RX while configuring */
    UART1_CR1 = (UART1_CR1 & (uint8_t)~(UART1_CR1_UARTD | UART1_CR1_M | UART1_CR1_PCEN));
    /* CR1: UARTD=0 (enabled), M=0 (8 data bits), PCEN=0 (no parity) -> 8N1 */
    UART1_CR1 |= UART1_CR1_UARTD;   /* actually UARTD=1 disables; keep 0 = enabled */
    UART1_CR1 &= (uint8_t)~UART1_CR1_UARTD;

    /* 115200 @ 16MHz: UARTDIV = 8.68 -> BRR1=0x08, BRR2=0x0B */
    UART1_BRR2 = 0x0B;          /* write BRR2 first */
    UART1_BRR1 = 0x08;          /* then BRR1 (latches divider) */

    /* CR3: stop bits 00 = 1 stop bit */
    UART1_CR3 &= (uint8_t)~(UART1_CR3_STOP1 | UART1_CR3_STOP0);

    /* CR2: TEN=1 transmitter enable, REN=1 receiver enable.
     * BUGFIX: REN was missing here (only TEN set) — receiver stayed
     * disabled, RXNE never set, RX IRQ never fired despite RIEN. */
    UART1_CR2 |= UART1_CR2_TEN | UART1_CR2_REN;

    /* Clear any residual flags by reading SR then DR (also clears OR).
     * Must be done BEFORE enabling RIEN, or a stale RXNE from reset noise
     * would fire a spurious RX interrupt on the first byte. */
    (void)UART1_SR;
    (void)UART1_DR;

    /* Publish self to the RX ISR, then enable RXNE interrupt:
     * REN already set above; RIEN=1 routes UART1_RX (vector 0x8050,
     * IRQ 18 -> __interrupt(18) in uart.c). Received bytes are stored
     * into self->rx_byte / self->rx_flag; the main loop echoes them
     * (ISR stays non-blocking). */
    s_rx_owner = self;
    UART1_CR2 |= UART1_CR2_RIEN;

    /* Clear the TX ring buffer: producers start from an empty queue,
     * tx_service (main loop) has nothing to drain until first write. */
    self->tx_head = 0;
    self->tx_tail = 0;
}

/* ========== UART1 RX Interrupt Helper ==========
 * STM8S103F3 UART1_RX vector = 0x8050 (IRQ 18).
 * NOTE: SDCC 4.5.0 stm8 后端只在"含 main 的编译单元"生成 __interrupt_vect
 * 向量表，且只收录该单元内定义的 __interrupt 函数。若本函数直接用
 * __interrupt(18) 定义在 uart.c，0x8050 不会有跳转条目（被 GSINIT 启动
 * 代码占用），RX 中断永不触发。因此这里改为普通处理函数，由 main 单元的
 * UART1_RX_IRQHandler __interrupt(18) 向量壳调用（见 led_blink_main.c）。
 * Non-blocking: read DR into the UART_t instance, set rx_flag,
 * leave the actual echo to the main loop. */
void uart_rx_irq_handler(void)
{
    UART_t *u = s_rx_owner;
    if (UART1_SR & UART1_SR_RXNE) {
        uint8_t b = UART1_DR;       /* reading DR clears RXNE (and OR) */
        if (u) {
            u->rx_byte = b;
            u->rx_flag = 1;
        }
    }
}

/* ========== TX Ring Buffer ==========
 * putc/puts/printf are now PRODUCERS ONLY: they copy bytes into the
 * software ring buffer and return immediately (never wait on TXE).
 * This makes every UART output call safe from ISR / callback context
 * (e.g. key event callbacks run inside the TIM1 ISR).
 *
 * Concurrency (single producer side / single consumer side):
 *   - tx_head is written only here (ISR/callback context)
 *   - tx_tail is written only in uart_tx_service (main loop)
 *   - both are uint8_t -> single-byte access is atomic on STM8, so a
 *     producer reading tail (full test) and a consumer reading head
 *     (empty test) never see a torn pointer.
 */

static void uart_putc(UART_t *self, char c)
{
    /* TX_BUF_SIZE must be a power of two: (head+1) & (SIZE-1) wraps
     * 127 -> 0. A plain uint8_t ++ would advance head to 128 after the
     * last slot and write tx_buf[128], which aliases tx_head/tx_tail and
     * the function pointers right after the buffer -> crash/reset after
     * ~120 chars of output. */
    uint8_t next = (uint8_t)((self->tx_head + 1) & (TX_BUF_SIZE - 1));
    if (next == self->tx_tail) {
        return;                 /* ring full: drop this char (debug output) */
    }
    self->tx_buf[self->tx_head] = (uint8_t)c;
    self->tx_head = next;       /* publish AFTER the byte is in the buffer */
}

/* ========== TX Drain (called from the main loop, blocking) ==========
 * Transmits every byte currently in the ring buffer. TXE wait happens
 * here in main context, so the ISR can still enqueue new bytes while we
 * drain (head advances, tail stays ours) without losing data or ticks.
 * If the UART clock / pin is broken, the per-byte timeout prevents the
 * main loop from hanging forever. */
static void uart_tx_service(UART_t *self)
{
    while (self->tx_head != self->tx_tail) {
        uint16_t guard = 0xFFFF;
        while (!(UART1_SR & UART1_SR_TXE)) {
            if (--guard == 0) {
                return;         /* give up: transmitter stuck */
            }
        }
        uint8_t b = self->tx_buf[self->tx_tail];
        self->tx_tail = (uint8_t)((self->tx_tail + 1) & (TX_BUF_SIZE - 1));
        UART1_DR = b;
    }
}

static void uart_puts(UART_t *self, const char *s)
{
    while (*s) {
        uart_putc(self, *s++);
    }
}

/* ========== Minimal printf-style formatter ========== */

static void uart_put_unsigned(UART_t *self, unsigned long v, int base)
{
    char buf[17];
    const char *digits = "0123456789ABCDEF";
    char *p = buf + sizeof(buf);
    if (v == 0) {
        uart_putc(self, '0');
        return;
    }
    *--p = '\0';
    while (v) {
        *--p = digits[v % base];
        v /= base;
    }
    uart_puts(self, p);
}

static void uart_put_signed(UART_t *self, long v)
{
    if (v < 0) {
        uart_putc(self, '-');
        v = -v;
    }
    uart_put_unsigned(self, (unsigned long)v, 10);
}

static void uart_vprintf(UART_t *self, const char *fmt, va_list ap)
{
    for (; *fmt; fmt++) {
        if (*fmt != '%') {
            uart_putc(self, *fmt);
            continue;
        }
        fmt++;                 /* skip '%' */
        switch (*fmt) {
        case 'c':
            uart_putc(self, (char)va_arg(ap, int));
            break;
        case 's': {
            const char *s = va_arg(ap, const char *);
            if (s) {
                uart_puts(self, s);
            }
            break;
        }
        case 'd':
            uart_put_signed(self, va_arg(ap, int));
            break;
        case 'u':
            uart_put_unsigned(self, va_arg(ap, unsigned int), 10);
            break;
        case 'x':
            uart_put_unsigned(self, va_arg(ap, unsigned int), 16);
            break;
        case '%':
            uart_putc(self, '%');
            break;
        default:
            uart_putc(self, '%');
            uart_putc(self, *fmt);
            break;
        }
    }
}

static void uart_printf(UART_t *self, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    uart_vprintf(self, fmt, ap);
    va_end(ap);
}

/* ========== UART Constructor ========== */

UART_t UART_Create(uint16_t base)
{
    UART_t uart;

    uart.base = base;
    uart.rx_byte = 0;
    uart.rx_flag = 0;

    /* Bind methods */
    uart.init        = uart_init;
    uart.putc        = uart_putc;
    uart.puts        = uart_puts;
    uart.printf      = uart_printf;
    uart.tx_service  = uart_tx_service;

    return uart;
}
