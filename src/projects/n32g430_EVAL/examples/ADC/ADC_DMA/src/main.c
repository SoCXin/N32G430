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
DMA_InitType DMA_InitStructure;
__IO uint16_t ADCConvertedValue;

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
    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();
    
    /* DMA channel1 configuration ----------------------------------------------*/
    DMA_Reset(DMA_CH1);
    DMA_InitStructure.PeriphAddr     = (uint32_t)&ADC->DAT;
    DMA_InitStructure.MemAddr        = (uint32_t)&ADCConvertedValue;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = 1;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_HALFWORD;
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_HALFWORD;
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_ENABLE;
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;
    DMA_Initializes(DMA_CH1, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH1, DMA_REMAP_ADC);
    /* Enable DMA channel1 */
    DMA_Channel_Enable(DMA_CH1);

    ADC_Initializes_Structure(&ADC_InitStructure);
    ADC_InitStructure.MultiChEn      = ENABLE;
    ADC_InitStructure.ContinueConvEn = ENABLE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_REGULAR_SWSTRRCH;
    ADC_InitStructure.ChsNumber      = ADC_REGULAR_LEN_1;
    ADC_Initializes(&ADC_InitStructure);
    
    /* ADC channel sampletime configuration */
    ADC_Channel_Sample_Time_Config(ADC_Channel_01_PA0, ADC_SAMP_TIME_55CYCLES5);
    /* ADC regular channel configuration */
    ADC_Regular_Sequence_Conversion_Number_Config(ADC_Channel_01_PA0, ADC_REGULAR_NUMBER_1);
    
    /* Enable ADC DMA */
    ADC_DMA_Transfer_Enable();

    /* Enable ADC */
    ADC_ON();
    
    /* Check ADC Ready */
    while(ADC_Flag_Status_Get(ADC_RD_FLAG, ADC_FLAG_JENDCA, ADC_FLAG_RDY) == RESET)
        ;
    
    /* Start ADC1 calibration */
    ADC_Calibration_Operation(ADC_CALIBRATION_ENABLE);
    /* Check the end of ADC1 calibration */
    while (ADC_Calibration_Operation(ADC_CALIBRATION_STS))
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
    
    /* Enable DMA GPIO ADC clocks */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA|RCC_AHB_PERIPH_GPIOA|RCC_AHB_PERIPH_GPIOB|RCC_AHB_PERIPH_ADC);
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

