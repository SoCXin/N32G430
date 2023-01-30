/* -----------------------------------------------------------------------------
 * Copyright (c) 2019 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty. 
 * In no event will the authors be held liable for any damages arising from 
 * the use of this software. Permission is granted to anyone to use this 
 * software for any purpose, including commercial applications, and to alter 
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not 
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be 
 *    appreciated but is not required. 
 * 
 * 2. Altered source versions must be plainly marked as such, and must not be 
 *    misrepresented as being the original software. 
 * 
 * 3. This notice may not be removed or altered from any source distribution.
 *   
 *
 * $Date:        15. April 2019
 * $Revision:    V1.00
 *  
 * Project:      Flash Device Description
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 
#include "FlashOS.h"        // FlashOS Structures

typedef volatile unsigned char  vu8;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;

#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

// Peripheral Memory Map
#define IWDG_BASE       0x40003000
#define FLASH_BASE      0x40022000

#define IWDG            ((IWDG_TypeDef *) IWDG_BASE)
#define FLASH           ((FLASH_TypeDef*) FLASH_BASE)

// FLASH BANK size
#define BANK1_SIZE      0x00020000      // Bank1 Size = 128kB
// System Memory size
#define SYSMEMORY_SIZE  0x00004000      // System Memory Size = 16kB

// Independent WATCHDOG
typedef struct {
  vu32 KR;
  vu32 PR;
  vu32 RLR;
  vu32 SR;
} IWDG_TypeDef;

// Flash Registers
typedef struct {
  vu32 ACR;                                     // offset  0x000
  vu32 KEYR;                                    // offset  0x004
  vu32 OPTKEYR;                                 // offset  0x008
  vu32 SR;                                      // offset  0x00C
  vu32 CR;                                      // offset  0x010
  vu32 AR;                                      // offset  0x014
  vu32 RESERVED0[1];
  vu32 OBR;                                     // offset  0x01C
  vu32 WRPR;                                    // offset  0x020
} FLASH_TypeDef;


// Flash Keys
#define RDPRT_KEY       0x5AA5
#define FLASH_KEY1      0x45670123
#define FLASH_KEY2      0xCDEF89AB

// Flash Control Register definitions
#define FLASH_PG        0x00000001
#define FLASH_PER       0x00000002

#define CR_PER_Reset    0x00001FFD

#define FLASH_MER       0x00000004
#define FLASH_OPTPG     0x00000010
#define FLASH_OPTER     0x00000020
#define FLASH_STRT      0x00000040
#define FLASH_LOCK      0x00000080
#define FLASH_OPTWRE    0x00000100

// Flash Status Register definitions
#define FLASH_BSY       0x00000001
#define FLASH_PGERR     0x00000004
#define FLASH_WRPRTERR  0x00000010
#define FLASH_EOP       0x00000020


unsigned long base_adr;


/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

#ifdef FLASH_MEM
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  base_adr = adr & ~(BANK1_SIZE - 1);          // Align to Size Boundary

  // Zero Wait State
  FLASH->ACR  = 0x00000000;

  // Unlock Flash    
  FLASH->KEYR  = FLASH_KEY1;
  FLASH->KEYR  = FLASH_KEY2;

  // Test if IWDG is running (IWDG in HW mode)
  if ((FLASH->OBR & 0x04) == 0x00) {
    // Set IWDG time out to ~32.768 second
    IWDG->KR  = 0x5555;                         // Enable write access to IWDG_PR and IWDG_RLR     
    IWDG->PR  = 0x06;                           // Set prescaler to 256  
    IWDG->RLR = 4095;                           // Set reload value to 4095
  }

  return (0);
}
#endif



/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {

  // Lock Flash
  FLASH->CR  |=  FLASH_LOCK;
  return (0);
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip (void) {

  FLASH->CR  |=  FLASH_MER;                     // Mass Erase Enabled
  FLASH->CR  |=  FLASH_STRT;                    // Start Erase

  while (FLASH->SR  & FLASH_BSY) {
    IWDG->KR = 0xAAAA;                          // Reload IWDG
  }

  FLASH->CR  &= ~FLASH_MER;                     // Mass Erase Disabled

  return (0);                                   // Done
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {

    FLASH->CR  |=  FLASH_PER;                   // Page Erase Enabled 
    FLASH->AR   =  adr;                         // Page Address
    FLASH->CR  |=  FLASH_STRT;                  // Start Erase

    while (FLASH->SR  & FLASH_BSY) {
      IWDG->KR = 0xAAAA;                        // Reload IWDG
    }

    FLASH->CR  &= ~FLASH_PER;                   // Page Erase Disabled 

  return (0);                                   // Done
}

/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */
int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

    IWDG->KR = 0xAAAA;

  sz = (sz + 1) & ~1;                           // Adjust size for Half Words
    while (sz) {

      FLASH->CR  |=  FLASH_PG;                  // Programming Enabled

      M32(adr) = *((unsigned int *)buf);      // Program Word
      while (FLASH->SR  & FLASH_BSY);

      FLASH->CR  &= ~FLASH_PG;                  // Programming Disabled

      // Check for Errors
      if (FLASH->SR  & (FLASH_PGERR | FLASH_WRPRTERR)) {
        FLASH->SR  |= FLASH_PGERR | FLASH_WRPRTERR;
        return (1);                             // Failed
      }

      // Go to next Half Word
      adr += 4;
      buf += 4;
      sz  -= 4;
    }
  return (0);                                   // Done
}
