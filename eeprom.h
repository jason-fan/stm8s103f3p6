/**
 * eeprom.h - STM8S103F3P6 data EEPROM helpers
 *
 * Target:   STM8S103F3 (medium density, 1KB data EEPROM 0x4000-0x43FF)
 * Compiler: SDCC
 *
 * Reading data EEPROM needs no unlock (it is mapped like RAM).
 * Writing requires: unlock via DUKR key sequence -> byte store ->
 * wait EOP -> relock. See RM0016 section on data EEPROM programming.
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

/* STM8S103 data EEPROM base address (1KB, 0x4000 - 0x43FF) */
#define EEPROM_BASE_ADDR  0x4000

/* Read uint16 (little-endian: low byte at addr, high byte at addr+1) */
uint16_t eeprom_read_u16(uint16_t addr);

/* Write uint16 (little-endian) with unlock/program/wait/relock sequence.
 * Returns 1 on success, 0 if unlock or programming timed out (caller
 * must NOT hang the system on failure - report and continue). */
uint8_t eeprom_write_u16(uint16_t addr, uint16_t val);

#endif /* EEPROM_H */
