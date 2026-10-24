/**
 * adc.c - STM8S103F3P6 ADC1 single conversion on PC4 (AIN2)
 *
 * Registers (STM8S103 datasheet DS5932, ADC1 base 0x5400; RM0016 bit map):
 *   ADC_CSR @ 0x5400  (EOC bit7, AWD bit6, EOCIE bit5, AWDIE bit4,
 *                      CH[3:0] channel select bits 3:0)
 *   ADC_CR1 @ 0x5401  (SPSEL[2:0] bits 6:4, CONT bit1, ADON bit0)
 *   ADC_CR2 @ 0x5402  (ALIGN bit3: 0 = right-aligned)
 *   ADC_DRH @ 0x5404  (data high byte: bits 9..2)
 *   ADC_DRL @ 0x5405  (data low bits 1..0)
 *
 * Channel mapping (low density, TSSOP20): AIN2 = PC4 (pin 14).
 * NOTE: PA1 is OSCIN only, NOT AIN1; channel 1 does not exist here.
 *
 * Single conversion (RM0016 24.5 / AN3023):
 *   - select the channel via CSR first (before power-on, as recommended)
 *   - first CR1.ADON=1 write powers the ADC up (0->1, no conversion)
 *   - second CR1.ADON=1 write (1->1) starts one conversion
 *   - wait EOC, then read DRH:DRL (EOC auto-clears on DR read)
 *
 * HISTORY OF THE FIX (2026-10-24):
 *   Previous code wrote "ADON" into ADC_CSR bit0, which is really CH[0]:
 *   that corrupted the selected channel from 2 to 3 (PD2/AIN3), so the
 *   reading never tracked PC4; and ADC_CR1=0x03 actually meant CONT=1,
 *   ADON=1, SPSEL=000 (/2), not "SPSEL=011" as the old comment claimed.
 */

#include "adc.h"

#define ADC_CSR  (*(volatile uint8_t *)0x5400)
#define ADC_CR1  (*(volatile uint8_t *)0x5401)
#define ADC_CR2  (*(volatile uint8_t *)0x5402)
#define ADC_DRH  (*(volatile uint8_t *)0x5404)
#define ADC_DRL  (*(volatile uint8_t *)0x5405)

#define ADC_CR1_ADON      0x01   /* bit0: ADC on / start conversion */
#define ADC_CR1_CONT      0x02   /* bit1: continuous conversion */
#define ADC_CR1_SPSEL     0x70   /* bits 6:4: prescaler selection */
#define ADC_CR1_SPSEL_DIV6  0x30 /* SPSEL=011 -> fADC=fMASTER/6 */
#define ADC_CR1_SPSEL_DIV8  0x40 /* SPSEL=100 -> fADC=fMASTER/8 = 2MHz@16MHz */

#define ADC_CSR_EOC  0x80   /* end of conversion */
#define ADC_CSR_CH   0x0F   /* channel select bits [3:0] */
#define ADC_CSR_STAT 0xF0   /* keep EOC/AWD/EOCIE/AWDIE when re-selecting */

#define CLK_PCKENR2       (*(volatile uint8_t *)0x50C8)  /* NOTE: 0x50C7 is PCKENR1! */
#define CLK_PCKENR2_ADC1  0x08   /* ADC1 clock enable (bit3) */

#define GPIO_PC      0x500A
#define PC_DDR       (*(volatile uint8_t *)(GPIO_PC + 2))
#define PC_CR1       (*(volatile uint8_t *)(GPIO_PC + 3))
#define PC_CR2       (*(volatile uint8_t *)(GPIO_PC + 4))

#define ADC_CH_AIN2    2     /* PC4 -> AIN2 */
#define ADC_EOC_TIMEOUT 100000  /* bounded poll budget, never hang */

void adc_init(void)
{
    /* Enable ADC1 clock (CLK_PCKENR2 bit3; reset default is on,
     * set explicitly for robustness). */
    CLK_PCKENR2 |= CLK_PCKENR2_ADC1;

    /* PC4 = analog input: DDR=0 (input), CR1=0 (no pull), CR2=0 */
    PC_DDR &= (uint8_t)~0x10;
    PC_CR1 &= (uint8_t)~0x10;
    PC_CR2 &= (uint8_t)~0x10;

    /* Select channel FIRST (CSR only; keep status bits). */
    ADC_CSR = (uint8_t)((ADC_CSR & ADC_CSR_STAT) | ADC_CH_AIN2);

    /* Single conversion (CONT=0), right-aligned (ALIGN=0),
     * SPSEL=100 -> fADC = fMASTER/8 = 2MHz @ 16MHz master.
     * Direct write while ADON=0: no conversion triggered. */
    ADC_CR1 = ADC_CR1_SPSEL_DIV8;
    ADC_CR2 = 0x00;               /* right-aligned */

    /* Power up ADC: first ADON write (0->1, does NOT start conversion). */
    ADC_CR1 |= ADC_CR1_ADON;
}

uint16_t adc_read_ch2(void)
{
    uint32_t i = 0;
    uint16_t result = 0;

    /* Re-assert channel: touch CSR only (keep EOC/AWD/EOCIE/AWDIE),
     * never touch CR1 here. */
    ADC_CSR = (uint8_t)((ADC_CSR & ADC_CSR_STAT) | ADC_CH_AIN2);

    /* Start conversion: second ADON=1 write (1->1) on CR1.
     * Note: RM0016 warns that ANY bit-manipulation on CR1 can start a
     * new conversion once ADON reads as 1; that is exactly what we want. */
    ADC_CR1 |= ADC_CR1_ADON;

    /* Wait EOC with bounded timeout (a few us at 2MHz fADC). */
    while (!(ADC_CSR & ADC_CSR_EOC)) {
        if (++i >= ADC_EOC_TIMEOUT) break;
    }

    /* Right-aligned 10-bit: DRH holds bits 9..2, DRL holds bits 1..0.
     * Reading DRH:DRL clears EOC automatically. */
    result = (uint16_t)(((uint16_t)ADC_DRH << 2) | (ADC_DRL & 0x03U));
    return result;
}

uint8_t adc_csr_raw(void)
{
    return ADC_CSR;
}
