/**
 * adc.h - STM8S103F3P6 ADC1 helper (single conversion, 10-bit)
 *
 * Target:   STM8S103F3 (ADC1, 10-bit)
 * Compiler: SDCC
 *
 * PC4 = AIN2 (channel 2), configured as analog input.
 * adc_read_ch2(): software-triggered single conversion, right-aligned
 * 10-bit result in [0..1023].
 *
 * NOTE: STM8S103F3 (low density, TSSOP20) has NO AIN0/AIN1 inputs.
 * PA1 is OSCIN only, NOT an ADC input. Available channels:
 *   AIN2=PC4(14), AIN3=PD2(19), AIN4=PD3(20), AIN5=PD5(2), AIN6=PD6(3)
 */

#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/* Initialise ADC1 for single conversion on AIN2 (PC4), right-aligned */
void adc_init(void);

/* Start one conversion on AIN2 and return the 10-bit result (0..1023).
 * Blocks until EOC (bounded timeout). Safe to call from the main loop. */
uint16_t adc_read_ch2(void);

/* Diagnostics: return raw ADC_CSR byte (ADON/EOC/channel bits). */
uint8_t adc_csr_raw(void);

#endif /* ADC_H */
