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

#define FLASH_PAGE_SIZE        ((uint16_t)0x800)
#define FLASH_WRITE_START_ADDR ((uint32_t)0x08008000)
#define FLASH_WRITE_END_ADDR   ((uint32_t)0x08010000)

int main(void)
{
    uint32_t Counter_Num = 0;
    uint32_t Erase_Data  = 0xCDEF89AB;
    
    /* USART Init */
    log_init();
    
    printf("Flash Program Test Start\r\n");
    /* Program FLASH */

    /* Unlocks the FLASH Program Erase Controller */
    FLASH_Unlock();

    /* Erase */
    if (FLASH_EOP != FLASH_One_Page_Erase(FLASH_WRITE_START_ADDR))
    {
        while(1)
        {
            printf("Flash EraseOnePage Error. Please Deal With This Error Promptly\r\n");
        }
    }

    /* Program */
    for (Counter_Num = 0; Counter_Num < FLASH_PAGE_SIZE; Counter_Num += 4)
    {
        if (FLASH_EOP != FLASH_Word_Program(FLASH_WRITE_START_ADDR + Counter_Num, Erase_Data))
        {
            while(1)
            {
                printf("Flash ProgramWord Error. Please Deal With This Error Promptly\r\n");
            }
        }
    }

    /* Locks the FLASH Program Erase Controller */
    FLASH_Lock();

    /* Check */
    for (Counter_Num = 0; Counter_Num < FLASH_PAGE_SIZE; Counter_Num += 4)
    {
        if (Erase_Data != (*(__IO uint32_t*)(FLASH_WRITE_START_ADDR + Counter_Num)))
        {
            printf("Flash Program Test Failed\r\n");
            break;
        }
    }

    printf("Flash Program Test End\r\n");

    while (1)
    {
    }   
}



