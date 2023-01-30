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
**/
#include "main.h"

/* N32G430_StdPeriph_Examples */

/* SPI_CRC */

#define BufferSize 32
#define SPICRC

SPI_InitType SPI_InitStructure;
uint16_t SPI_Master_Buffer_Tx[BufferSize] = {0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E, 0x0F10,
                                             0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E, 0x1F20,
                                             0x2122, 0x2324, 0x2526, 0x2728, 0x292A, 0x2B2C, 0x2D2E, 0x2F30,
                                             0x3132, 0x3334, 0x3536, 0x3738, 0x393A, 0x3B3C, 0x3D3E, 0x3F40};
uint16_t SPI_Slave_Buffer_Tx[BufferSize] = {0x5152, 0x5354, 0x5556, 0x5758, 0x595A, 0x5B5C, 0x5D5E, 0x5F60,
                                            0x6162, 0x6364, 0x6566, 0x6768, 0x696A, 0x6B6C, 0x6D6E, 0x6F70,
                                            0x7172, 0x7374, 0x7576, 0x7778, 0x797A, 0x7B7C, 0x7D7E, 0x7F80,
                                            0x8182, 0x8384, 0x8586, 0x8788, 0x898A, 0x8B8C, 0x8D8E, 0x8F90};
uint16_t SPI_Master_Buffer_Rx[BufferSize], SPI_Slave_Buffer_Rx[BufferSize];
uint32_t TxIdx = 0, RxIdx = 0;
__IO uint16_t CRC1Value = 0, CRC2Value = 0;

volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;

void RCC_Configuration(void);
void GPIO_Configuration(void);
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

    /* System clocks configuration ---------------------------------------------*/
    RCC_Configuration();
    /* GPIO configuration ------------------------------------------------------*/
    GPIO_Configuration();
    /* log configuration ------------------------------------------------------*/
    log_init();
    log_info("This is SPI CRC demo!\r\n");

    SPI_I2S_Reset(SPI_MASTER);
    SPI_I2S_Reset(SPI_SLAVE);

    /* SPI_MASTER configuration --------------------------------------------------*/
    SPI_Initializes_Structure(&SPI_InitStructure);
    SPI_InitStructure.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;
    SPI_InitStructure.SpiMode       = SPI_MODE_MASTER;
    SPI_InitStructure.DataLen       = SPI_DATA_SIZE_16BITS;
    SPI_InitStructure.CLKPOL        = SPI_CLKPOL_LOW;
    SPI_InitStructure.CLKPHA        = SPI_CLKPHA_SECOND_EDGE;
    SPI_InitStructure.NSS           = SPI_NSS_SOFT;
    /* It is recommended that the SPI master mode of the C version chips should not exceed 18MHz */
    SPI_InitStructure.BaudRatePres  = SPI_BR_PRESCALER_8;
    SPI_InitStructure.FirstBit      = SPI_FB_MSB;
    SPI_InitStructure.CRCPoly       = 7;
    SPI_Initializes(SPI_MASTER, &SPI_InitStructure);
    SPI_Set_Nss_Level(SPI_MASTER, SPI_NSS_HIGH);

    /* SPI_SLAVE configuration --------------------------------------------------*/
    SPI_InitStructure.SpiMode = SPI_MODE_SLAVE;
    SPI_Initializes(SPI_SLAVE, &SPI_InitStructure);
    SPI_Set_Nss_Level(SPI_SLAVE, SPI_NSS_LOW);

    /* Enable SPI_MASTER CRC calculation */
    SPI_CRC_Enable(SPI_MASTER);
    /* Enable SPI_SLAVE CRC calculation */
    SPI_CRC_Enable(SPI_SLAVE);

#ifdef HARDWARE_NSS
        SPI_NSS_Config(SPI_MASTER, SPI_NSS_HARD);
        SPI_SS_Output_Enable(SPI_MASTER);
        SPI_NSS_Config(SPI_SLAVE, SPI_NSS_HARD);
#endif

    /* Enable SPI_SLAVE */
    SPI_ON(SPI_SLAVE);
    /* Enable SPI_MASTER */
    SPI_ON(SPI_MASTER);

    /* Transfer procedure */
    while (TxIdx < BufferSize - 1)
    {
        /* Wait for SPI_MASTER Tx buffer empty */
        while (SPI_I2S_Flag_Status_Get(SPI_MASTER, SPI_I2S_FLAG_TE) == RESET)
            ;
        /* Send SPI_SLAVE data */
        SPI_I2S_Data_Transmit(SPI_SLAVE, SPI_Slave_Buffer_Tx[TxIdx]);
        /* Send SPI_MASTER data */
        SPI_I2S_Data_Transmit(SPI_MASTER, SPI_Master_Buffer_Tx[TxIdx++]);
        /* Wait for SPI_SLAVE data reception */
        while (SPI_I2S_Flag_Status_Get(SPI_SLAVE, SPI_I2S_FLAG_RNE) == RESET)
            ;
        /* Read SPI_SLAVE received data */
        SPI_Slave_Buffer_Rx[RxIdx] = SPI_I2S_Data_Get(SPI_SLAVE);
        /* Wait for SPI_MASTER data reception */
        while (SPI_I2S_Flag_Status_Get(SPI_MASTER, SPI_I2S_FLAG_RNE) == RESET)
            ;
        /* Read SPI_MASTER received data */
        SPI_Master_Buffer_Rx[RxIdx++] = SPI_I2S_Data_Get(SPI_MASTER);
    }
    /* Wait for SPI_MASTER Tx buffer empty */
    while (SPI_I2S_Flag_Status_Get(SPI_MASTER, SPI_I2S_FLAG_TE) == RESET)
        ;
    /* Wait for SPI_SLAVE Tx buffer empty */
    while (SPI_I2S_Flag_Status_Get(SPI_SLAVE, SPI_I2S_FLAG_TE) == RESET)
        ;
    /* Send last SPI_Slave_Buffer_Tx data */
    SPI_I2S_Data_Transmit(SPI_SLAVE, SPI_Slave_Buffer_Tx[TxIdx]);
    /* Enable SPI_SLAVE CRC transmission */
    SPI_Next_Transmit_CRC(SPI_SLAVE);
    /* Send last SPI_Master_Buffer_Tx data */
    SPI_I2S_Data_Transmit(SPI_MASTER, SPI_Master_Buffer_Tx[TxIdx]);
    /* Enable SPI_MASTER CRC transmission */
    SPI_Next_Transmit_CRC(SPI_MASTER);

    /* Wait for SPI_MASTER last data reception */
    while (SPI_I2S_Flag_Status_Get(SPI_MASTER, SPI_I2S_FLAG_RNE) == RESET)
        ;
    /* Read SPI_MASTER last received data */
    SPI_Master_Buffer_Rx[RxIdx] = SPI_I2S_Data_Get(SPI_MASTER);

    /* Wait for SPI_SLAVE last data reception */
    while (SPI_I2S_Flag_Status_Get(SPI_SLAVE, SPI_I2S_FLAG_RNE) == RESET)
        ;
    /* Read SPI_SLAVE last received data */
    SPI_Slave_Buffer_Rx[RxIdx] = SPI_I2S_Data_Get(SPI_SLAVE);
        
    /* Wait for SPI_MASTER data reception: CRC transmitted by SPI_SLAVE */
    while (SPI_I2S_Flag_Status_Get(SPI_MASTER, SPI_I2S_FLAG_RNE) == RESET)
        ;
    /* Wait for SPI_SLAVE data reception: CRC transmitted by SPI_MASTER */
    while (SPI_I2S_Flag_Status_Get(SPI_SLAVE, SPI_I2S_FLAG_RNE) == RESET)
        ;

    /* Check the received data with the send ones */
    TransferStatus1 = Buffercmp(SPI_Slave_Buffer_Rx, SPI_Master_Buffer_Tx, BufferSize);
    TransferStatus2 = Buffercmp(SPI_Master_Buffer_Rx, SPI_Slave_Buffer_Tx, BufferSize);
    /* TransferStatus1, TransferStatus2 = PASSED, if the data transmitted and received
       are correct */
    /* TransferStatus1, TransferStatus2 = FAILED, if the data transmitted and received
       are different */

    /* Test on the SPI_MASTER CRC Error flag */
    if ((SPI_I2S_Flag_Status_Get(SPI_MASTER, SPI_FLAG_CRCERR)) == SET)
    {
        TransferStatus2 = FAILED;
    }

    /* Test on the SPI_SLAVE CRC Error flag */
    if ((SPI_I2S_Flag_Status_Get(SPI_SLAVE, SPI_FLAG_CRCERR)) == SET)
    {
        TransferStatus1 = FAILED;
    }

    if(TransferStatus1 == PASSED && TransferStatus2 == PASSED)
    {
        log_info("Test PASS!\r\n");
    }
    else
    {
        log_info("Test ERR!\r\n");
    }
    /* Read SPI_MASTER received CRC value */
    CRC1Value = SPI_I2S_Data_Get(SPI_MASTER);
    /* Read SPI_SLAVE received CRC value */
    CRC2Value = SPI_I2S_Data_Get(SPI_SLAVE);
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
    /* spi clock enable */
    RCC_APB2_Peripheral_Clock_Enable(SPI_MASTER_PERIPH | SPI_SLAVE_PERIPH);

    /* GPIO Periph clock enable */
    RCC_AHB_Peripheral_Clock_Enable(SPI_MASTER_PERIPH_GPIO | SPI_SLAVE_PERIPH_GPIO);
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
    /* Configure master pins: SCK, MISO and MOSI ---------------------------------*/
    /* Confugure SCK and MOSI pins as Alternate Function Push Pull */
    GPIO_InitStructure.Pin        = SPI_MASTER_MOSI_PIN | SPI_MASTER_CLK_PIN | SPI_MASTER_MISO_PIN | SPI_MASTER_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Alternate = SPI_MASTER_GPIO_ALTERNATE;
    GPIO_Peripheral_Initialize(SPI_MASTER_GPIO, &GPIO_InitStructure);

    /* Configure slave pins: SCK, MISO and MOSI ---------------------------------*/
    /* Confugure SCK and MOSI pins as Input Floating */
    GPIO_InitStructure.Pin        = SPI_SLAVE_CLK_PIN | SPI_SLAVE_MOSI_PIN | SPI_SLAVE_MISO_PIN | SPI_SLAVE_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = SPI_SLAVE_GPIO_ALTERNATE;
    GPIO_Peripheral_Initialize(SPI_SLAVE_GPIO, &GPIO_InitStructure);
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


