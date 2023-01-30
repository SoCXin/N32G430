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
#include "n32g430.h"
#include "log.h"

GPIO_InitType GPIO_InitStructure;
RCC_ClocksType RCC_ClockFreq;
ErrorStatus HSEStartUpStatus;
ErrorStatus HSIStartUpStatus;

void NVIC_Configuration(void);

#define SYSCLK_SOURCE_HSI 1
#define SYSCLK_SOURCE_HSE 2
#define SYSCLK_SOURCE_HSI_PLL 3
#define SYSCLK_SOURCE_HSE_PLL 4

#ifndef SYSCLK_SOURCE_SELECT
#define SYSCLK_SOURCE_SELECT SYSCLK_SOURCE_HSE_PLL /*select sysclk source */
#endif


#define SYSCLK_USE_HSIDIV2_PLL 0
#define SYSCLK_USE_HSEDIV2_PLL 1
#define HSE_Value   (8000000)

void SetSysClockToHSI(void);
void SetSysClockToHSE(void);
void SetSysClockToPLL(uint32_t freq, uint8_t src);
/**
*\*\name    PrintfClockInfo.
*\*\fun     Printf clock information.
*\*\param   none
*\*\return  none 
**/
void PrintfClockInfo(const char* msg)
{
    uint32_t TimeDelay = 0xFFFF;
    /* reinit after sysclk changed */
    log_init(); 
    /* Wait for the configuration to succeed */
    while(--TimeDelay) 
    {
    }
    log_info("--------------------------------\n");
    log_info("%s:\n", msg);
    RCC_Clocks_Frequencies_Value_Get(&RCC_ClockFreq);
    log_info("SYSCLK: %d\n", RCC_ClockFreq.SysclkFreq);
    log_info("HCLK: %d\n", RCC_ClockFreq.HclkFreq);
    log_info("PCLK1: %d\n", RCC_ClockFreq.Pclk1Freq);
    log_info("PCLK2: %d\n", RCC_ClockFreq.Pclk2Freq);
}

/**
*\*\name    main.
*\*\fun     main function.
*\*\param   none
*\*\return  none 
**/
int main(void)
{
    
    log_init();
    log_info("-----------------\nRCC_ClockConfig Demo.\n");

    PrintfClockInfo("After reset"); 
    
#if SYSCLK_SOURCE_SELECT == SYSCLK_SOURCE_HSI
    
    SetSysClockToHSI();
    PrintfClockInfo("HSI, 8MHz");
    
#elif SYSCLK_SOURCE_SELECT == SYSCLK_SOURCE_HSE  
    
    SetSysClockToHSE();
    PrintfClockInfo("HSE, 8MHz");
    
#elif SYSCLK_SOURCE_SELECT == SYSCLK_SOURCE_HSI_PLL 
    
    SetSysClockToPLL(48000000, SYSCLK_USE_HSIDIV2_PLL);
    PrintfClockInfo("HSIDIV2->PLL, 48M");
    
    
#elif SYSCLK_SOURCE_SELECT == SYSCLK_SOURCE_HSE_PLL 
   
    SetSysClockToPLL(32000000, SYSCLK_USE_HSEDIV2_PLL);
    PrintfClockInfo("HSEDIV2->PLL, 32M");
    
#endif

    /* Enable Clock Security System(CSS): this will generate an NMI exception
       when HSE clock fails */
    RCC_Clock_Security_System_Enable();

    /* NVIC configuration
     * ------------------------------------------------------*/
    NVIC_Configuration();

    /* Output HSE clock on MCO pin
     * ---------------------------------------------*/
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin             = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode       = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF9_MCO;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    
    RCC_MCO_Source_Config(RCC_MCO_SYSCLK);

   
    while (1);
}

/**
*\*\name    SetSysClockToHSI.
*\*\fun     Selects HSI as System clock source and configure HCLK, PCLK2
*\*\         and PCLK1 prescalers.
*\*\param   none
*\*\return  none 
**/
void SetSysClockToHSI(void)
{
    RCC_Reset();

    RCC_HSI_Enable();

    /* Wait till HSI is ready */
    HSIStartUpStatus = RCC_HSI_Stable_Wait();

    if (HSIStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        FLASH_Prefetch_Buffer_Enable();

        /* Flash 0 wait state */
        FLASH_Latency_Set(FLASH_LATENCY_0);

        /* HCLK = SYSCLK */
        RCC_Hclk_Config(RCC_SYSCLK_DIV1);

        /* PCLK2 = HCLK */
        RCC_Pclk2_Config(RCC_HCLK_DIV1);

        /* PCLK1 = HCLK */
        RCC_Pclk1_Config(RCC_HCLK_DIV1);

        /* Select HSI as system clock source */
        RCC_Sysclk_Config(RCC_SYSCLK_SRC_HSI);
           
        /* Wait till HSI is used as system clock source */
        while (RCC_Sysclk_Source_Get() != RCC_CFG_SCLKSTS_HSI)
        {
        }
    }
    else
    {
        /* If HSI fails to start-up, the application will have wrong clock
           configuration. User can add here some code to deal with this error */

        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

/**
*\*\name    SetSysClockToHSE.
*\*\fun     Selects HSE as System clock source and configure HCLK, PCLK2
*\*\          and PCLK1 prescalers.
*\*\param   none
*\*\return  none 
**/
void SetSysClockToHSE(void)
{
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration
     * -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_Reset();

    /* Enable HSE */
    RCC_HSE_Config(RCC_HSE_ENABLE);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_HSE_Stable_Wait();

    if (HSEStartUpStatus == SUCCESS)
    {
        /* Enable Prefetch Buffer */
        FLASH_Prefetch_Buffer_Enable();

        if (HSE_Value <= 18000000)
        {
            /* Flash 0 wait state */
            FLASH_Latency_Set(FLASH_LATENCY_0);
        }
        else
        {
            /* Flash 1 wait state */
            FLASH_Latency_Set(FLASH_LATENCY_1);
        }

        /* HCLK = SYSCLK */
        RCC_Hclk_Config(RCC_SYSCLK_DIV1);

        /* PCLK2 = HCLK */
        RCC_Pclk2_Config(RCC_HCLK_DIV1);

        /* PCLK1 = HCLK */
        RCC_Pclk1_Config(RCC_HCLK_DIV1);

        /* Select HSE as system clock source */
        RCC_Sysclk_Config(RCC_SYSCLK_SRC_HSE);
       
        /* Wait till HSE is used as system clock source */
        while (RCC_Sysclk_Source_Get() != RCC_CFG_SCLKSTS_HSE)
        {
        }
    }
    else
    {
        /* If HSE fails to start-up, the application will have wrong clock
           configuration. User can add here some code to deal with this error */

        /* Go to infinite loop */
        while (1)
        {
        }
    }
}

/**
*\*\name    SetSysClockToPLL.
*\*\fun     Selects PLL clock as System clock source and configure HCLK, PCLK2
*\*\         and PCLK1 prescalers.
*\*\param   none
*\*\return  none 
**/
void SetSysClockToPLL(uint32_t freq, uint8_t src)
{
    uint32_t pllmul;
    uint32_t latency;
    uint32_t pclk1div, pclk2div;

    if (HSE_VALUE != 8000000)
    {
        /* HSE_VALUE == 8000000 is needed in this project! */
        while (1);
    }

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration
     * -----------------------------*/
    /* RCC system reset(for debug purpose) */
    RCC_Reset();

    if (src == SYSCLK_USE_HSIDIV2_PLL) 
    {
        /* Enable HSI */
        RCC_HSI_Enable();

        /* Wait till HSI is ready */
        HSIStartUpStatus = RCC_HSI_Stable_Wait();

        if (HSIStartUpStatus != SUCCESS)
        {
            /* If HSI fails to start-up, the application will have wrong clock
               configuration. User can add here some code to deal with this
               error */

            /* Go to infinite loop */
            while (1);
        }

    }
    else if (src == SYSCLK_USE_HSEDIV2_PLL) 
    {
        /* Enable HSE */
        RCC_HSE_Config(RCC_HSE_ENABLE);

        /* Wait till HSE is ready */
        HSEStartUpStatus = RCC_HSE_Stable_Wait();

        if (HSEStartUpStatus != SUCCESS)
        {
            /* If HSE fails to start-up, the application will have wrong clock
               configuration. User can add here some code to deal with this
               error */

            /* Go to infinite loop */
            while (1);
        }

    }

    switch (freq)
    {
        case 32000000:
            latency  = FLASH_LATENCY_0;
            if(src == SYSCLK_USE_HSIDIV2_PLL)
            {
                pllmul = RCC_PLL_MUL_8;
            }
            else if(src == SYSCLK_USE_HSEDIV2_PLL)
            {
                pllmul = RCC_PLL_MUL_8;
            }
            pclk1div = RCC_HCLK_DIV2;
            pclk2div = RCC_HCLK_DIV1;
            break;
        case 48000000:
            latency  = FLASH_LATENCY_1;
            if(src == SYSCLK_USE_HSIDIV2_PLL)
            {
              pllmul = RCC_PLL_MUL_12;
            }
            else if(src == SYSCLK_USE_HSEDIV2_PLL)
            {
               pllmul = RCC_PLL_MUL_12;
            }
            pclk1div = RCC_HCLK_DIV2;
            pclk2div = RCC_HCLK_DIV1;
            break;
        default:
            while (1);
    }

    FLASH_Latency_Set(latency);

    /* HCLK = SYSCLK */
    RCC_Hclk_Config(RCC_SYSCLK_DIV1);

    /* PCLK2 = HCLK */
    RCC_Pclk2_Config(pclk2div);

    /* PCLK1 = HCLK */
    RCC_Pclk1_Config(pclk1div);
    
    if(src == SYSCLK_USE_HSEDIV2_PLL)
    {
        RCC_PLL_Config(RCC_PLL_SRC_HSE_DIV2,pllmul);
    }
    else
    {
        RCC_PLL_Config(RCC_PLL_SRC_HSI_DIV2,pllmul);
    }

    /* Enable PLL */
    RCC_PLL_Enable();

    /* Wait till PLL is ready */
   // while (RCC_Flag_Status_Get(RCC_FLAG_PLLRD) == RESET);
     /* Wait till PLL is ready */
    while ((RCC->CTRL & RCC_CTRL_PLLRDF) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC_Sysclk_Config(RCC_SYSCLK_SRC_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_Sysclk_Source_Get() != RCC_CFG_SCLKSTS_PLL);
}

/**
*\*\name    NVIC_Configuration.
*\*\fun     NVIC Configuration.
*\*\param   none
*\*\return  none 
**/
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable and configure RCC global IRQ channel */
    NVIC_InitStructure.NVIC_IRQChannel                   = RCC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}


