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
 */

#include "main.h"
#include "n32g430.h"
#include "log.h"

#include <string.h>

#define I2C1_DR_ADDR        0x40005410
#define I2C2_DR_ADDR        0x40005810
#define I2C1_SLAVE_ADDRESS7 0x30
#define I2C2_SLAVE_ADDRESS7 0x30
#define BUFFER_SIZE         8
#define I2C_CLK_SPEED       100000

I2C_InitType I2C_InitStructure;
DMA_InitType DMA_InitStructure;
uint8_t I2C1_Tx_Buffer[BUFFER_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8};
uint8_t I2C2_Rx_Buffer[BUFFER_SIZE];
uint8_t Tx_Idx = 0, Rx_Idx = 0;

/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
    log_init();

    log_info("----------------------\nDMA I2C to RAM Demo.\n");

    /* System Clocks Configuration */
    RCC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();
    
    /* DMA channel5 configuration */
    DMA_Reset(DMA_CH5);
    DMA_InitStructure.PeriphAddr     = (uint32_t)I2C2_DR_ADDR;
    DMA_InitStructure.MemAddr        = (uint32_t)I2C2_Rx_Buffer;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = BUFFER_SIZE;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_BYTE;
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGHEST;
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;
    DMA_Initializes(DMA_CH5, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH5,DMA_REMAP_I2C2_RX);

    /* DMA channel6 configuration */
    DMA_Reset(DMA_CH6);
    DMA_InitStructure.PeriphAddr = (uint32_t)I2C1_DR_ADDR;
    DMA_InitStructure.MemAddr    = (uint32_t)I2C1_Tx_Buffer;
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.Priority   = DMA_CH_PRIORITY_HIGH;
    DMA_Initializes(DMA_CH6, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH6,DMA_REMAP_I2C1_TX);

    I2C_Reset(I2C1);

    /* I2C1 configuration */
    I2C_InitStructure.BusMode     = I2C_BUSMODE_I2C;
    I2C_InitStructure.DutyCycle   = I2C_FMDUTYCYCLE_2;
    I2C_InitStructure.OwnAddr1    = I2C1_SLAVE_ADDRESS7;
    I2C_InitStructure.AckEnable   = I2C_ACKEN;
    I2C_InitStructure.AddrMode    = I2C_ADDR_MODE_7BIT;
    I2C_InitStructure.ClkSpeed    = I2C_CLK_SPEED;
    I2C_Initializes(I2C1, &I2C_InitStructure);
    /* I2C2 configuration */
    I2C_InitStructure.OwnAddr1 = I2C2_SLAVE_ADDRESS7;
    I2C_Initializes(I2C2, &I2C_InitStructure);

    /* Enable I2C1 and I2C2 */
    I2C_ON(I2C1);
    I2C_ON(I2C2);

    /*----- Transmission Phase -----*/
    /* Send I2C1 START condition */
    I2C_Generate_Start_Enable(I2C1);
    /* Test on I2C1 EV5 and clear it */
    while (!I2C_Event_Check(I2C1, I2C_EVT_MASTER_MODE_FLAG))
        ;
    /* Send I2C2 slave Address for write */
    I2C_7bit_Addr_Send(I2C1, I2C2_SLAVE_ADDRESS7, I2C_DIRECTION_SEND);
    /* Test on I2C2 EV1 and clear it */
    while (!I2C_Event_Check(I2C2, I2C_EVT_SLAVE_RECV_ADDR_MATCHED))
        ;
    /* Test on I2C1 EV6 and clear it */
    while (!I2C_Event_Check(I2C1, I2C_EVT_MASTER_TXMODE_FLAG))
        ;

    /* Enable I2C2 DMA */
    I2C_DMA_Transfer_Enable(I2C2);
    /* Enable I2C1 DMA */
    I2C_DMA_Transfer_Enable(I2C1);

    /* Enable DMA Channel5 */
    DMA_Channel_Enable(DMA_CH5);
    /* Enable DMA Channel6 */
    DMA_Channel_Enable(DMA_CH6);

    /* DMA1 Channel5 transfer complete test */
    while (!DMA_Flag_Status_Get(DMA, DMA_CH5_TXCF))
        ;
    /* DMA1 Channel6 transfer complete test */
    while (!DMA_Flag_Status_Get(DMA, DMA_CH6_TXCF))
        ;

    /* Send I2C1 STOP Condition */
    I2C_Generate_Stop_Enable(I2C1);
    /* Test on I2C2 EV4 */
    while (!I2C_Event_Check(I2C2, I2C_EVT_SLAVE_STOP_RECVD))
        ;
    /* Clear I2C2 STOPF flag: read operation to I2C_STS1 followed by a
    write operation to I2C_CTRL1 */
    (void)(I2C_Flag_Status_Get(I2C2, I2C_FLAG_STOPF));
    I2C_ON(I2C2);

    /* Check if the transmitted and received data are equal */
    if (memcmp(I2C1_Tx_Buffer, I2C2_Rx_Buffer, BUFFER_SIZE) == 0)
    {
        log_info("DMA I2C to RAM passed\n");
    }
    else
    {
        log_error("DMA I2C to RAM failed\n");
    }

    while (1)
    {
    }
}

/**
*\*\name    RCC_Configuration.
*\*\fun     Configures the different system clocks.
*\*\param   none
*\*\return  none
**/
void RCC_Configuration(void)
{
    /* Enable DMA clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);
    /* Enable GPIOB clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
    /* Enable I2C1 and I2C2 clock */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_I2C1 | RCC_APB1_PERIPH_I2C2);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
}

/**
*\*\name    GPIO_Configuration.
*\*\fun     onfigures the different GPIO ports.
*\*\param   none
*\*\return  none
**/
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Configure I2C1 pins: SCL and SDA */
    GPIO_InitStructure.Pin            = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_I2C1;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

    /* Configure I2C2 pins: SCL and SDA */
    GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF7_I2C2;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
}


