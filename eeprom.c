/**
 * eeprom.c - STM8S103F3P6 data EEPROM helpers
 *
 * Registers (medium density STM8S103, RM0016):
 *   FLASH_IAPSR @ 0x5054  (EOP bit2, DUL bit6)
 *   FLASH_DUKR  @ 0x5060  (data EEPROM unlock key register)
 *
 * Data EEPROM programming (standard mode):
 *   1. unlock : write 0xAE then 0x56 to FLASH_DUKR -> DUL=1
 *   2. program: normal byte store to the EEPROM address
 *   3. wait   : until EOP=1 (end of programming)
 *   4. relock : clear DUL bit in FLASH_IAPSR (or reset) -> DUL=0
 */

#include "eeprom.h"

#define FLASH_IAPSR  (*(volatile uint8_t *)0x505F)
#define FLASH_DUKR   (*(volatile uint8_t *)0x5064)

#define IAPSR_EOP   0x04   /* end of programming (bit2) */
#define IAPSR_DUL   0x08   /* data EEPROM unlocked (bit3) */

#define EEPROM_TIMEOUT  2000  /* bounded polling budget (ms at 16MHz) */

/* Wait for a FLASH_IAPSR flag with a bounded polling budget so a
 * failed unlock/program can NEVER hang the application forever.
 * Returns 1 if the flag appeared, 0 on timeout. */
static uint8_t eeprom_wait_flag(uint8_t mask)
{
    uint16_t i;
    for (i = 0; i < EEPROM_TIMEOUT; i++) {
        if (FLASH_IAPSR & mask) return 1;
    }
    return 0;
}

/* Tiny delay between the two DUKR key writes (belt & braces) */
static void delay_short(void)
{
    __asm__("nop"); __asm__("nop"); __asm__("nop"); __asm__("nop"); __asm__("nop");
    __asm__("nop"); __asm__("nop"); __asm__("nop"); __asm__("nop"); __asm__("nop");
}

uint16_t eeprom_read_u16(uint16_t addr)
{
    uint8_t *p = (uint8_t *)addr;
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

uint8_t eeprom_write_u16(uint16_t addr, uint16_t val)
{
    uint8_t *p = (uint8_t *)addr;

    /* Unlock data EEPROM (skip if already unlocked this session) */
    if (!(FLASH_IAPSR & IAPSR_DUL)) {
        FLASH_DUKR = 0xAE;      /* first hardware key */
        delay_short();
        FLASH_DUKR = 0x56;      /* second hardware key */
        if (!eeprom_wait_flag(IAPSR_DUL)) return 0;  /* unlock failed */
    }

    p[0] = (uint8_t)(val & 0xFF);
    if (!eeprom_wait_flag(IAPSR_EOP)) return 0;

    p[1] = (uint8_t)(val >> 8);
    if (!eeprom_wait_flag(IAPSR_EOP)) return 0;

    /* Relock data EEPROM: clear DUL bit */
    FLASH_IAPSR = (uint8_t)(FLASH_IAPSR & (uint8_t)~IAPSR_DUL);
    return 1;
}
