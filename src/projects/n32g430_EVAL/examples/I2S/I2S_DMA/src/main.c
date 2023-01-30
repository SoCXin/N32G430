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
*\*\file      main.c
*\*\author    Nations
*\*\version   v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved. 
*/
#include "main.h"

/* N32G430_StdPeriph_Examples */

#define BufferSize 32

I2S_InitType I2S_InitStructure;

uint16_t I2S_Master_Buffer_Tx[BufferSize] = {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
                                             0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
                                             0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
                                             0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40};
uint16_t I2S_Slave_Buffer_Tx[BufferSize] = {0x4142, 0x4344, 0x4546, 0x4748, 0x494A, 0x4B4C, 0x4D4E, 0x4F50,
                                            0x5152, 0x5354, 0x5556, 0x5758, 0x595A, 0x5B5C, 0x5D5E, 0x5F60,
                                            0x6162, 0x6364, 0x6566, 0x6768, 0x696A, 0x6B6C, 0x6D6E, 0x6F70,
                                            0x7172, 0x7374, 0x7576, 0x7778, 0x797A, 0x7B7C, 0x7D7E, 0x7F90};
uint16_t I2S_Master_Buffer_Rx[BufferSize], I2S_Slave_Buffer_Rx[BufferSize];

volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;

void RCC_Configuration(void);
void GPIO_Configuration(void);
void DMA_Configuration(void);
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength);


/*  Main program. */
int main(void)
{
    /* At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_N32G430.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_N32G430.c file
     */
    /* System clocks configuration */
    RCC_Configuration();
    /* GPIO configuration */
    GPIO_Configuration();
    /* DMA configuration */
    DMA_Configuration();
    /* log configuration */
    log_init();    
    log_info("This is I2S DMA demo!\r\n");
    /* I2S_MASTER configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_MASTER_TX;
    I2S_InitStructure.AudioFrequency = I2S_AUDIO_FREQ_11K;
    I2S_InitStructure.CLKPOL = I2S_CLKPOL_HIGH;
    I2S_InitStructure.DataFormat = I2S_DATA_FMT_16BITS;
    I2S_InitStructure.MCLKEnable = I2S_MCLK_ENABLE;
    I2S_InitStructure.Standard = I2S_STD_PCM_LONGFRAME;
    I2S_Initializes(I2S_MASTER, &I2S_InitStructure);
    /* I2S_SLAVE configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_SlAVE_RX;
    I2S_Initializes(I2S_SLAVE, &I2S_InitStructure);
    /* Enable the I2S DMA*/
    SPI_I2S_DMA_Transfer_Enable(I2S_SLAVE, SPI_I2S_DMA_RX);
    SPI_I2S_DMA_Transfer_Enable(I2S_MASTER, SPI_I2S_DMA_TX); 
    /* Enable I2S */
    I2S_ON(I2S_SLAVE);
    I2S_ON(I2S_MASTER);
    /* Wait for the transmission to complete */
    while(DMA_Flag_Status_Get(DMA, DMA_CH1_TXCF) == RESET)
        ;
    while(DMA_Flag_Status_Get(DMA, DMA_CH4_TXCF) == RESET)
        ;

    /* Check the received data with the send ones */
    TransferStatus1 = Buffercmp(I2S_Slave_Buffer_Rx, I2S_Master_Buffer_Tx, BufferSize);
    /* Disable the I2S */
    I2S_OFF(I2S_MASTER);
    I2S_OFF(I2S_SLAVE);
    /* I2S_MASTER configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_MASTER_RX;
    I2S_Initializes(I2S_MASTER, &I2S_InitStructure);
    /* I2S_SLAVE configuration */
    I2S_InitStructure.I2sMode = I2S_MODE_SlAVE_TX;
    I2S_Initializes(I2S_SLAVE, &I2S_InitStructure);
    /* Enable the I2S DMA*/
    SPI_I2S_DMA_Transfer_Enable(I2S_SLAVE, SPI_I2S_DMA_TX);
    SPI_I2S_DMA_Transfer_Enable(I2S_MASTER, SPI_I2S_DMA_RX); 
    /* Enable I2S */
    I2S_ON(I2S_SLAVE);
    I2S_ON(I2S_MASTER);
    /* Wait for the transmission to complete */
    while(DMA_Flag_Status_Get(DMA, DMA_CH2_TXCF) == RESET)
        ;
    while(DMA_Flag_Status_Get(DMA, DMA_CH3_TXCF) == RESET)
        ;
    /* Check the received data with the send ones */
    TransferStatus2 = Buffercmp(I2S_Master_Buffer_Rx, I2S_Slave_Buffer_Tx, BufferSize);
    /* Disable the I2S */
    I2S_OFF(I2S_MASTER);
    I2S_OFF(I2S_SLAVE);
    if(TransferStatus1 == PASSED && TransferStatus2 == PASSED)
    {
        log_info("Test PASS!\r\n");
    }
    else
    {
        log_info("Test ERR!\r\n");
    }
    while(1)
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
    /* PCLK2 = HCLK/2 */
    RCC_Pclk2_Config(RCC_HCLK_DIV2);

    /* Enable peripheral clocks --------------------------------------------------*/
    /* I2S clock enable */
    RCC_APB2_Peripheral_Clock_Enable(I2S_MASTER_PERIPH | I2S_SLAVE_PERIPH);

    /* GPIO Periph clock enable */
    RCC_AHB_Peripheral_Clock_Enable(I2S_MASTER_PERIPH_GPIO | I2S_SLAVE_PERIPH_GPIO | RCC_AHB_PERIPH_DMA);
}

/**
*\*\name    GPIO_Configuration.
*\*\fun     Configures the different GPIO ports.
*\*\param   none
*\*\return  none
**/
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Configure master pins: MCK, SD, CK and WS */
    /* Confugure all pins as Alternate Function Push Pull */
    GPIO_InitStructure.Pin        = I2S_MASTER_MCK_PIN | I2S_MASTER_SD_PIN | I2S_MASTER_CK_PIN | I2S_MASTER_WS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Alternate = I2S_MASTER_GPIO_ALTERNATE;
    GPIO_Peripheral_Initialize(I2S_MASTER_GPIO, &GPIO_InitStructure);
  
    GPIO_InitStructure.Pin = I2S_MASTER_MCK_PIN;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_Peripheral_Initialize(I2S_MASTER_GPIO, &GPIO_InitStructure);
  
    /* Configure slave pins: MCK, SD, CK and WS */
    /* Confugure all pins as Input Floating */
    GPIO_InitStructure.Pin        = I2S_SLAVE_MCK_PIN | I2S_SLAVE_SD_PIN | I2S_SLAVE_CK_PIN | I2S_SLAVE_WS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = I2S_SLAVE_GPIO_ALTERNATE;
    GPIO_Peripheral_Initialize(I2S_SLAVE_GPIO, &GPIO_InitStructure);
}

/**
*\*\name    DMA_Configuration.
*\*\fun     Configures the different DMA channel.
*\*\param   none
*\*\return  none
**/
void DMA_Configuration(void)
{
    DMA_InitType DMA_InitStructure;
  /* Reset the DMA channels */
    DMA_Reset(DMA_CH1);
    DMA_Reset(DMA_CH2);
    DMA_Reset(DMA_CH3);
    DMA_Reset(DMA_CH4);

    /* I2S_MASTER TX DMA config */
    DMA_InitStructure.MemAddr = (uint32_t)&I2S_Master_Buffer_Tx[0];
    DMA_InitStructure.MemDataSize = DMA_MEM_DATA_WIDTH_HALFWORD;
    DMA_InitStructure.MemoryInc = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.PeriphAddr = (uint32_t)&I2S_MASTER->DAT;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_HALFWORD;
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.BufSize = BufferSize;
    DMA_InitStructure.CircularMode = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Mem2Mem = DMA_MEM2MEM_DISABLE;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_MEDIUM;
    DMA_Initializes(DMA_CH1, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH1, I2S_MASTER_DMA_TX_CH);
    
    /* I2S_MASTER RX DMA config */
    DMA_InitStructure.MemAddr = (uint32_t)&I2S_Master_Buffer_Rx[0];
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGHEST;
    DMA_Initializes(DMA_CH2, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH2, I2S_MASTER_DMA_RX_CH);
    
    /* I2S_Slave TX DMA config */
    DMA_InitStructure.MemAddr = (uint32_t)&I2S_Slave_Buffer_Tx[0];
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.PeriphAddr = (uint32_t)&I2S_SLAVE->DAT;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGH;
    DMA_Initializes(DMA_CH3, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH3, I2S_SLAVE_DMA_TX_CH);
    
    /* I2S_MASTER RX DMA config */
    DMA_InitStructure.MemAddr = (uint32_t)&I2S_Slave_Buffer_Rx[0];
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGHEST;
    DMA_Initializes(DMA_CH4, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH4, I2S_SLAVE_DMA_RX_CH);
    /* Enable the I2S Tx and Rx DMA channels */
    DMA_Channel_Enable(DMA_CH1);
    DMA_Channel_Enable(DMA_CH2);
    DMA_Channel_Enable(DMA_CH3);
    DMA_Channel_Enable(DMA_CH4);
}

/**
*\*\name    Buffercmp.
*\*\fun     Compares two buffers.
*\*\param   pBuffer1
*\*\param   pBuffer2
*\*\param   BufferLength
*\*\return  FAILED or PASSED
**/
TestStatus Buffercmp(uint16_t* pBuffer1, uint16_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }
        pBuffer1++;
        pBuffer2++;
    }
    return PASSED;
}

#ifdef USE_FULL_ASSERT

/**
*\*\name    assert_failed.
*\*\fun     Reports the name of the source file and the source line number
*\*\                where the assert_param error has occurred
*\*\param   expr
*\*\param   file        pointer to the source file name
*\*\param   line        assert_param error line source number
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
