/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
*\*\file main.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/


#include "main.h"
#include "log.h"
#include <stdio.h>

/* RAM address */
#define RAM_BASE    ((uint32_t)0x20000000)
/* RAM bitband address */
#define RAM_BB_BASE ((uint32_t)0x22000000)

__IO uint32_t Data = 0, DataAddr = 0, DataBitValue = 0;

#define Data_ResetBit_BB(DataAddr, BitNumber)                                                                          \
    (*(__IO uint32_t*)(RAM_BB_BASE | ((DataAddr - RAM_BASE) << 5) | ((BitNumber) << 2)) = 0)

#define Data_SetBit_BB(DataAddr, BitNumber)                                                                            \
    (*(__IO uint32_t*)(RAM_BB_BASE | ((DataAddr - RAM_BASE) << 5) | ((BitNumber) << 2)) = 1)

#define Data_GetBit_BB(DataAddr, BitNumber) (Data >> BitNumber)

int main(void)
{
    uint8_t i;
    
    /* USART Init */
    log_init();
    
    printf("Cortex-M4F BitBand \r\n");
    
    /* A mapping formula shows how to reference each word in the alias region to a
       corresponding bit in the bit-band region. The mapping formula is:
       bit_byte_addr = bit_band_base + (byte_offset x 32) + (bit_number x 4)

       Parameter Description:
         bit_byte_addr: The address of the byte in the alias memory region that maps to the targeted bit.
         bit_band_base: The starting address of the alias region
         byte_offset:   The number of the byte in the bit-band region that contains the targeted bit
         bit_number:    The bit position (0-7) of the targeted bit */

    /* Get the variable address */
    DataAddr = (uint32_t)&Data;

    /* Modify variable bit using bit-band access */

    while (1)
    {
        for (i = 0; i < 8; i++)
        {
            /* Modify Data variable bit i */
            Data_SetBit_BB(DataAddr, i); /* Data = (0x00000001 << i) */
            printf("%d SetBit Data = 0x%08X \r\n", i, DataBitValue);
            /* Get Data variable bit i value */
            DataBitValue = Data_GetBit_BB(DataAddr, i); /* DataBitValue = 0x00000001 */
            printf("%d SetBit DataBitValue = 0x%08X \r\n", i, DataBitValue);

            Data_ResetBit_BB(DataAddr, i); /* Data = 0x00000000 */
            printf("%d ResetBit Data = 0x%08X \r\n", i, DataBitValue);
            /* Get Data variable bit i value */
            DataBitValue = Data_GetBit_BB(DataAddr, i); /* DataBitValue = 0x00000000 */
            printf("%d ResetBit DataBitValue = 0x%08X \r\n", i, DataBitValue);
        }
    }
  
}



