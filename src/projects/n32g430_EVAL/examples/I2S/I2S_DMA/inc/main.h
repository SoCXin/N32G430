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
*\*\file      main.h
*\*\author    Nations
*\*\version   v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved. 
*/
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g430_spi.h"
#include "n32g430_dma.h"
#include "log.h"

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status, TestStatus;

#define I2S_MASTER                SPI1
#define I2S_MASTER_PERIPH         RCC_APB2_PERIPH_SPI1
#define I2S_MASTER_PERIPH_GPIO    RCC_AHB_PERIPH_GPIOA
#define I2S_MASTER_GPIO           GPIOA
#define I2S_MASTER_GPIO_ALTERNATE GPIO_AF1_SPI1
#define I2S_MASTER_MCK_PIN        GPIO_PIN_6
#define I2S_MASTER_SD_PIN         GPIO_PIN_7
#define I2S_MASTER_CK_PIN         GPIO_PIN_5
#define I2S_MASTER_WS_PIN         GPIO_PIN_4
#define I2S_MASTER_DMA_TX_CH      DMA_REMAP_SPI1_TX
#define I2S_MASTER_DMA_RX_CH      DMA_REMAP_SPI1_RX

#define I2S_SLAVE                 SPI2
#define I2S_SLAVE_PERIPH          RCC_APB2_PERIPH_SPI2
#define I2S_SLAVE_PERIPH_GPIO     RCC_AHB_PERIPH_GPIOB
#define I2S_SLAVE_GPIO            GPIOB
#define I2S_SLAVE_GPIO_ALTERNATE  GPIO_AF1_SPI2
#define I2S_SLAVE_MCK_PIN         GPIO_PIN_14
#define I2S_SLAVE_SD_PIN          GPIO_PIN_15
#define I2S_SLAVE_CK_PIN          GPIO_PIN_13
#define I2S_SLAVE_WS_PIN          GPIO_PIN_12
#define I2S_SLAVE_DMA_TX_CH       DMA_REMAP_SPI2_TX
#define I2S_SLAVE_DMA_RX_CH       DMA_REMAP_SPI2_RX



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */


