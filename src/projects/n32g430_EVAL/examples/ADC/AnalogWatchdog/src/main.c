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


ADC_InitType ADC_InitStructure;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);

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

    /* ADC Configuration ------------------------------------------------------*/
    ADC_InitStructure.MultiChEn      = ENABLE;
    ADC_InitStructure.ContinueConvEn = ENABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_REGULAR_SWSTRRCH;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = ADC_REGULAR_LEN_1;
    ADC_Initializes(&ADC_InitStructure);

    /* ADC regular channel2 configuration */
    ADC_Channel_Sample_Time_Config(ADC_Channel_02_PA1, ADC_SAMP_TIME_55CYCLES5);
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_02_PA1, ADC_REGULAR_NUMBER_1);

    /* Configure high and low analog watchdog thresholds */
    ADC_Analog_Watchdog_HighThresholds_Config(0x0B00);
    ADC_Analog_Watchdog_LowThresholds_Config(0x0300);
    /* Configure channel2 as the single analog watchdog guarded channel */
    ADC_Analog_Watchdog_Mode_Channel_Config(ADC_ANALOG_WTDG_SINGLE_MODE ,ADC_Channel_02_PA1);
    
    /* Enable Analog Watchdog */
    ADC_Analog_Watchdog_Enable(ADC_ANALOG_WTDG_REGULAR);
    /* Enable AWD interrupt */
    ADC_Interrupts_Enable(ADC_INT_AWD);
    
    /* Enable ADC */
    ADC_ON();
    
    /* Check ADC Ready */
    while(ADC_Flag_Status_Get(ADC_RD_FLAG, ADC_FLAG_JENDCA, ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC1 calibration */
    ADC_Calibration_Operation(ADC_CALIBRATION_ENABLE);
    /* Check the end of ADC1 calibration */
    while (ADC_Calibration_Operation(ADC_CALIBRATION_STS) == SET)
        ;

    /* Start ADC Software Conversion */
    ADC_Regular_Channels_Software_Conversion_Operation(ADC_EXTRTRIG_SWSTRRCH_ENABLE);
    while (1)
    {

    }
}

/**
*\*\name    RCC_Configuration.
*\*\fun     Configures the different system clocks.
*\*\return  none
**/
void RCC_Configuration(void)
{
    /* Enable peripheral clocks ------------------------------------------------*/
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA|RCC_AHB_PERIPH_GPIOA|RCC_AHB_PERIPH_GPIOB|
                                    RCC_AHB_PERIPH_GPIOC|RCC_AHB_PERIPH_GPIOD|RCC_AHB_PERIPH_ADC);
    /* RCC_ADCHCLK_DIV16*/
    ADC_Clock_Mode_Config(ADC_CKMOD_AHB, RCC_ADCHCLK_DIV16);
    RCC_ADC_1M_Clock_Config(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8);  //selsect HSE as RCC ADC1M CLK Source
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
    /* Configure adc input as analog input -------------------------*/
    GPIO_InitStructure.Pin       = GPIO_PIN_7 |GPIO_PIN_6 |GPIO_PIN_5 |GPIO_PIN_4 |GPIO_PIN_3 |GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin       = GPIO_PIN_14 |GPIO_PIN_13 |GPIO_PIN_12 |GPIO_PIN_11 |GPIO_PIN_10 |GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_ANALOG;
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
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC_IRQn;
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

