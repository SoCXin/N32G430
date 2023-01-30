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

void RCC_Configuration(void);
void GPIO_Configuration(void);
void COMP_Configuratoin(void);
void NVIC_Configuration(void);
void ChangeVmVp(void);


/**
*\*\name    Main program.
*\*\return  none
**/
int main(void)
{
    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();

    /* NVIC configuration ------------------------------------------------------*/
    NVIC_Configuration();

    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();

    /* COMP configuration ------------------------------------------------------*/
    COMP_Configuratoin();
    while (1)
    {
        ChangeVmVp();
    }
}

/**
*\*\name    ChangeVmVp.
*\*\fun     Self Generate Puls ,by skip line connect to vp and vm if need.
*\*\return  none
**/
void ChangeVmVp(void)
{
    GPIO_Pins_Set(GPIOB, GPIO_PIN_11);
    GPIO_PBSC_Pins_Reset(GPIOB, GPIO_PIN_12);
    {
        uint32_t i = 0;
        while (i++ < 10000)
            ;
    }
    GPIO_PBSC_Pins_Reset(GPIOB, GPIO_PIN_11);
    GPIO_Pins_Set(GPIOB, GPIO_PIN_12);
    {
        uint32_t i = 0;
        while (i++ < 10000)
            ;
    }
}

/**
*\*\name    COMP_Configuratoin.
*\*\fun     Configures the comp module.
*\*\return  none
**/
void COMP_Configuratoin(void)
{
    COMP_InitType COMP_Initial;

    /*Set dac3,dac2,dac1. */
    COMP_Voltage_Reference_Config(63, true, 32, true, 16, true);
    /*Initial comp*/
    COMP_Initializes_Structure(&COMP_Initial);
    COMP_Initial.InpSel     = COMP1_INPSEL_PB10;
    COMP_Initial.InmSel     = COMP1_INMSEL_PA5;
    COMP_Initial.SampWindow = 18;       //(0~31)
    COMP_Initial.Threshold  = 12;       //Thresh should be greater than half of SampWindow and should be less than SampWindow at the same time.
    COMP_Initializes(COMP1, &COMP_Initial);
    /*enable comp1*/
    COMP_ON(COMP1);
}

/**
*\*\name    RCC_Configuration.
*\*\fun     Configures the different system clocks.
*\*\return  none
**/
void RCC_Configuration(void)
{
    /* Enable COMP clocks */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_COMP | RCC_APB1_PERIPH_COMP_FILT);
    /* Enable GPIOA, GPIOB, GPIOC and GPIOD clocks */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA | RCC_AHB_PERIPH_GPIOB | RCC_AHB_PERIPH_GPIOC | RCC_AHB_PERIPH_GPIOD);
}

/**
*\*\name    GPIO_Configuration.
*\*\fun     Configures the different GPIO ports.
*\*\return  none
**/
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /*INP*/
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    GPIO_InitStructure.Pin        = GPIO_PIN_10;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
    /*INM*/
    GPIO_InitStructure.Pin = GPIO_PIN_5;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    /*OUT*/
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pin       = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF8_COMP1;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);

    /*PB11,P12 as connect to INP,INM by external skip line*/
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT_PP;
    GPIO_InitStructure.Pin       = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
}
/**
*\*\name    NVIC_Configuration.
*\*\fun     Configures Vector Table base location.
*\*\return  none
**/
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Configure and enable ADC interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = COMP_1_2_3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}

#ifdef USE_FULL_ASSERT
/**
*\*\name    assert_failed.
*\*\fun     Reports the name of the source file and the source line number
*\*\        where the assert_param error has occurred.
*\*\param   file pointer to the source file name
*\*\param   line assert_param error line source number
*\*\return  none
**/
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

