/**
 * adc.c - STM8S103F3 ADC1 Driver Implementation
 *
 * Single-conversion mode: init selects channel + SPSEL,
 * read starts one conversion and returns the 10-bit result.
 */

#include <stdint.h>
#include "adc.h"

/* ========== GPIO Register Access ========== */

#define GPIO_REG(port, off)     (*(volatile uint8_t *)((port) + (off)))
#define GPIO_DDR(port)          GPIO_REG(port, 2)
#define GPIO_CR1(port)          GPIO_REG(port, 3)
#define GPIO_CR2(port)          GPIO_REG(port, 4)

/* STM8S103 GPIO port base addresses */
#define GPIO_PC                 0x500A
#define GPIO_PD                 0x500F

/* ========== ADC Register Bit Masks ========== */

#define ADC_CSR_EOC             (uint8_t)0x80
#define ADC_CR1_ADON            (uint8_t)0x01
#define ADC_CR1_CONT            (uint8_t)0x02
#define ADC_CR1_SPSEL           (uint8_t)0x70

/* ========== ADC1 Peripheral Clock Gate ==========
 * STM8S103: ADC1 clock is gated by CLK_PCKENR2 bit3 (ADC1EN, 0x08).
 * (git HEAD / pre-OO code used 0x08; 0x01 was a wrong port of the
 *  mask — it would enable a different peripheral, not ADC1.)
 * uart.c only opens PCKENR1 (UART/TIM1/etc); without PCKENR2 bit3,
 * every ADC1 register write is ignored, ADON never starts a conversion,
 * EOC never sets -> adc_read() would spin forever. */
#define CLK_PCKENR2             (*(volatile uint8_t *)0x50C8)
#define CLK_PCKENR2_ADC1EN      (uint8_t)0x08

/* EOC poll timeout: prevents adc_read() from hanging the main loop
 * forever if the conversion never completes (clock gate closed / pin
 * misconfigured). ~5000 polls is far beyond a normal 2-8us conversion. */
#define ADC_EOC_TIMEOUT         5000U

/* ========== Static Helpers ========== */

/* Resolve AIN channel -> GPIO port base address */
static uint16_t adc_ch_port(uint8_t ch)
{
    switch (ch) {
        case ADC_CH_PC4:  return GPIO_PC;   /* AIN2 = PC4 */
        case ADC_CH_PD2:  return GPIO_PD;   /* AIN3 = PD2 */
        case ADC_CH_PD3:  return GPIO_PD;   /* AIN4 = PD3 */
        case ADC_CH_PD5:  return GPIO_PD;   /* AIN5 = PD5 */
        case ADC_CH_PD6:  return GPIO_PD;   /* AIN6 = PD6 */
        default:           return 0;        /* invalid channel */
    }
}

/* Resolve AIN channel -> GPIO pin number */
static uint8_t adc_ch_pin(uint8_t ch)
{
    switch (ch) {
        case ADC_CH_PC4:  return 4;
        case ADC_CH_PD2:  return 2;
        case ADC_CH_PD3:  return 3;
        case ADC_CH_PD5:  return 5;
        case ADC_CH_PD6:  return 6;
        default:           return 0;
    }
}

/* ========== Method Implementations ========== */

static void adc_init(ADC_t *self)
{
    uint16_t port = adc_ch_port(self->ch);
    uint8_t  mask = (uint8_t)(1 << adc_ch_pin(self->ch));

    if (port == 0) {
        return;   /* invalid channel, nothing to configure */
    }

    /* Enable ADC1 peripheral clock FIRST (PCKENR2 bit0).
     * Without this, all ADC register writes below are silently ignored. */
    CLK_PCKENR2 |= CLK_PCKENR2_ADC1EN;

    /* GPIO -> input floating: DDR=0, CR1=0, CR2=0 */
    GPIO_DDR(port) &= (uint8_t)~mask;
    GPIO_CR1(port) &= (uint8_t)~mask;
    GPIO_CR2(port) &= (uint8_t)~mask;

    /* CSR: select channel, preserve status bits (incl. EOC) */
    ADC1_CSR = (ADC1_CSR & (uint8_t)0xF0) | (self->ch & 0x0F);

    /* CR1: set SPSEL (mask 0x70), clear CONT (single conversion) */
    ADC1_CR1 = (ADC1_CR1 & (uint8_t)~ADC_CR1_SPSEL)
             | ((self->spsel << 4) & ADC_CR1_SPSEL);
    ADC1_CR1 &= (uint8_t)~ADC_CR1_CONT;

    /* Power up ADC now (ADON 0->1, no conversion triggered).
     * Without this the FIRST adc_read() only powers up on its ADON
     * write and the first EOC poll can time out (sentinel 0xFFFF). */
    ADC1_CR1 |= ADC_CR1_ADON;
}

/* ========== Single Conversion (internal) ==========
 * STM8S ADC1 (unlike STM32) starts a new single-mode conversion on a
 * simple WRITE of 1 to ADON — no 0->1 edge required. Keep ADON high
 * (ADC stays powered) between reads: clearing ADON first would power
 * the ADC down and back up on every sample, and the re-powered
 * conversion often never sets EOC -> timeout -> constant 0V readings.
 * Returns 0xFFFF sentinel if EOC never sets within the timeout. */
static uint16_t adc_convert_once(void)
{
    uint16_t guard = 0;

    /* Clear EOC before starting a new conversion */
    ADC1_CSR &= (uint8_t)~ADC_CSR_EOC;
    /* Write 1 to ADON: start a new conversion (ADC stays powered) */
    ADC1_CR1 |= ADC_CR1_ADON;
    /* Wait for conversion complete (EOC), with timeout so a broken ADC
     * clock / channel can never hang the main loop (which would also
     * stop beep.update() and uart.tx_service() -> beep never stops). */
    while ((ADC1_CSR & ADC_CSR_EOC) == 0) {
        if (++guard >= ADC_EOC_TIMEOUT) {
            return 0xFFFF;      /* timeout sentinel */
        }
    }
    /* Merge 10-bit result (right-aligned): DRH holds bits 9..2,
     * DRL holds bits 1..0. Correct formula: (DRH << 2) | (DRL & 0x03).
     * NOTE: DO NOT use (DRH << 8 | DRL) & 0x3FF — that keeps only the
     * top 2 + bottom 2 bits and silently drops the middle 6 bits,
     * e.g. full-scale 1021 (0b1111111101) becomes 769 (0b1100000001). */
    return (uint16_t)(((uint16_t)ADC1_DRH << 2) | (ADC1_DRL & 0x03U));
}

/* ========== Settling Delay ==========
 * STM8 ADC1 samples its input for a FIXED 1 ADC clock cycle
 * (~0.56us at SPSEL=7) - there is no programmable sampling time.
 * Before starting a conversion we pause briefly so the external
 * input network (potentiometer wiper, wiring, contact resistance,
 * stray capacitance) has time to charge the ADC sample capacitor.
 * Volatile loop on a 16MHz core: roughly 0.5-1ms. */
static void adc_settle_delay(void)
{
    volatile uint16_t i;
    for (i = 0; i < 3000; i++) {
        __asm__("nop");
    }
}

static uint16_t adc_read(ADC_t *self)
{
    uint32_t sum = 0;
    uint16_t v;
    uint8_t  i, n = (self->samples < 1) ? 1 : self->samples;

    /* Settle the input before sampling (see adc_settle_delay) */
    adc_settle_delay();

    /* Average n consecutive conversions; a single-shot reading on an
     * unstable input (pot wiper / wiring / contact resistance) jumps
     * between quantization levels (e.g. 512/256), averaging restores
     * a stable value close to the true DC level. */
    for (i = 0; i < n; i++) {
        v = adc_convert_once();
        if (v == 0xFFFF) {
            return 0x0000;      /* timeout sentinel -> report 0V */
        }
        sum += v;
    }
    return (uint16_t)((sum + (n >> 1)) / n);   /* rounded average */
}

static void adc_set_samples(ADC_t *self, uint8_t n)
{
    self->samples = (n < 1) ? 1 : n;
}

static void adc_power_down(ADC_t *self)
{
    (void)self;
    /* ADON = 0: power down ADC */
    ADC1_CR1 &= (uint8_t)~ADC_CR1_ADON;
}

/* ========== Constructor ========== */

ADC_t ADC_Create(uint8_t ch, uint8_t spsel)
{
    ADC_t adc;

    adc.ch        = ch;
    adc.spsel     = spsel;
    adc.samples   = ADC_DEFAULT_SAMPLES;   /* 8 = settle delay + averaging */
    adc.init      = adc_init;
    adc.read      = adc_read;
    adc.setSamples= adc_set_samples;
    adc.powerDown = adc_power_down;

    return adc;
}
