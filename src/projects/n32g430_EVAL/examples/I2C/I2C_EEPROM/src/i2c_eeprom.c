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
*\*\file i2c_eeprom.c
*\*\author Nations 
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "n32g430.h"
#include "n32g430_dma.h"
#include "i2c_eeprom.h"
#include "string.h"
#include "stdbool.h"

/** addtogroup I2C_EEPROM **/

/* when EEPROM is writing data inside,it won't response the request from the master.check the ack,
if EEPROM response,make clear that EEPROM finished the last data-writing,allow the next operation */
static bool check_begin = FALSE;
static bool OffsetDone  = FALSE;

u32 sEETimeout = sEE_LONG_TIMEOUT;

static u8 MasterDirection = Transmitter;
static u16 SlaveADDR;
static u16 DeviceOffset = 0x0;

u16 I2C_NumByteToWrite   = 0;
u8 I2C_NumByteWritingNow = 0;
u8* I2C_pBuffer          = 0;
u16 I2C_WriteAddr        = 0;

static u8 SendBuf[8]          = {0};
static u16 BufCount           = 0;
static u16 Int_NumByteToWrite = 0;
static u16 Int_NumByteToRead  = 0;
/* global state variable i2c_comm_state */
volatile I2C_STATE i2c_comm_state;

/**
*\*\name    sEE_TIMEOUT_UserCallback.
*\*\fun     Timeout callback used by the I2C EEPROM driver.
*\*\param   none
*\*\return  none 
**/
u8 sEE_TIMEOUT_UserCallback(void)
{
    printf("error!!!\r\n");
    /* Block communication and all processes */
    while (1)
    {
    }
}

/**
*\*\name    I2C_EE_Init.
*\*\fun     Initializes I2C\GPIO\NVIC\RCC used by the I2C EEPROM driver.
*\*\param   none
*\*\return  none 
**/
void I2C_EE_Init(void)
{
    /** GPIO configuration and clock enable */
    GPIO_InitType GPIO_InitStructure;
    I2C_InitType I2C_InitStructure;
#if PROCESS_MODE == 1 // interrupt
    NVIC_InitType NVIC_InitStructure;
#endif
   
    /** enable peripheral clk*/
    I2Cx_peripheral_clk_en();
    I2C_Reset(I2Cx);

    I2Cx_scl_clk_en();
    I2Cx_sda_clk_en();
    
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = I2Cx_SCL_PIN | I2Cx_SDA_PIN;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_I2C1;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_Peripheral_Initialize(GPIOx, &GPIO_InitStructure);

    /** I2C periphral configuration */
    I2C_Reset(I2Cx);
    I2C_InitStructure.BusMode     = I2C_BUSMODE_I2C;
    I2C_InitStructure.DutyCycle   = I2C_FMDUTYCYCLE_2;
    I2C_InitStructure.OwnAddr1    = 0xff;
    I2C_InitStructure.AckEnable   = I2C_ACKEN;
    I2C_InitStructure.AddrMode    = I2C_ADDR_MODE_7BIT;
    I2C_InitStructure.ClkSpeed    = I2C_Speed;
    I2C_Initializes(I2Cx, &I2C_InitStructure);
#if PROCESS_MODE == 1 /* interrupt */
    /** I2C NVIC configuration */
    NVIC_Priority_Group_Set(NVIC_PER0_SUB4_PRIORITYGROUP);
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
#endif
}

/**
*\*\name  I2C_EE_WriteBuffer.
*\*\fun   Writes buffer of data to the I2C EEPROM.
*\*\param pBuffer pointer to the buffer  containing the data to be
*\*\                  written to the EEPROM.
*\*\param WriteAddr EEPROM's internal address to write to.
*\*\param NumByteToWrite number of bytes to write to the EEPROM.
**/
void I2C_EE_WriteBuffer(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
    if (I2C_Flag_Status_Get(I2Cx, I2C_FLAG_BUSY))
    {
        return;
    }
    I2C_pBuffer        = pBuffer;
    I2C_WriteAddr      = WriteAddr;
    I2C_NumByteToWrite = NumByteToWrite;

    while (I2C_NumByteToWrite > 0)
    {
        I2C_EE_WriteOnePage(I2C_pBuffer, I2C_WriteAddr, I2C_NumByteToWrite);
    }
}

/**
*\*\name  I2C_EE_WriteOnePage.
*\*\fun   Writes a page of data to the I2C EEPROM, general called by
*\*\         I2C_EE_WriteBuffer.
*\*\param pBuffer pointer to the buffer  containing the data to be
*\*\                  written to the EEPROM.
*\*\param WriteAddr EEPROM's internal address to write to.
*\*\param NumByteToWrite number of bytes to write to the EEPROM.
**/
void I2C_EE_WriteOnePage(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
    u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0;

    Addr        = WriteAddr % I2C_PageSize;
    count       = I2C_PageSize - Addr;
    NumOfPage   = NumByteToWrite / I2C_PageSize;
    NumOfSingle = NumByteToWrite % I2C_PageSize;

    I2C_NumByteWritingNow = 0;
    /** If WriteAddr is I2C_PageSize aligned */
    if (Addr == 0)
    {
        /** If NumByteToWrite < I2C_PageSize */
        if (NumOfPage == 0)
        {
            I2C_NumByteWritingNow = NumOfSingle;
            I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        }
        /** If NumByteToWrite > I2C_PageSize */
        else
        {
            I2C_NumByteWritingNow = I2C_PageSize;
            I2C_EE_PageWrite(pBuffer, WriteAddr, I2C_PageSize);
        }
    }
    /** If WriteAddr is not I2C_PageSize aligned */
    else
    {
        /* If NumByteToWrite < I2C_PageSize */
        if (NumOfPage == 0)
        {
            I2C_NumByteWritingNow = NumOfSingle;
            I2C_EE_PageWrite(pBuffer, WriteAddr, NumOfSingle);
        }
        /* If NumByteToWrite > I2C_PageSize */
        else
        {
            if (count != 0)
            {
                I2C_NumByteWritingNow = count;
                I2C_EE_PageWrite(pBuffer, WriteAddr, count);
            }
        }
    }
}

/**
*\*\name  I2C_EE_PageWrite.
*\*\fun   Writes more than one byte to the EEPROM with a single WRITE
*\*\         cycle. The number of byte can't exceed the EEPROM page size.
*\*\param pBuffer pointer to the buffer containing the data to be
*\*\                  written to the EEPROM.
*\*\param WriteAddr EEPROM's internal address to write to (1-16).
*\*\param NumByteToWrite number of bytes to write to the EEPROM.
**/
void I2C_EE_PageWrite(u8* pBuffer, u16 WriteAddr, u16 NumByteToWrite)
{
#if PROCESS_MODE == 0 /* polling */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_Flag_Status_Get(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send START condition */
    I2C_Generate_Start_Enable(I2Cx);
    /** Test on EV5 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send EEPROM address for write */
    I2C_7bit_Addr_Send(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_SEND);
    /** Test on EV6 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_TXMODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send the EEPROM's internal address to write to */
    I2C_Data_Send(I2Cx, WriteAddr);
    /** Test on EV8 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** While there is data to be written */
    while (NumByteToWrite--)
    {
        /** Send the current byte */
        I2C_Data_Send(I2Cx, *pBuffer);
        /** Point to the next byte to be written */
        pBuffer++;
        /** Test on EV8 and clear it */
        sEETimeout = sEE_LONG_TIMEOUT;
        while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
        {
            if ((sEETimeout--) == 0)
                sEE_TIMEOUT_UserCallback();
        }
    }
    /** Send STOP condition */
    I2C_Generate_Stop_Enable(I2Cx);
    I2C_EE_WaitEepromStandbyState();
    I2C_EE_WriteOnePageCompleted();

#elif PROCESS_MODE == 1 /* interrupt */
    /** initialize static parameter */
    MasterDirection = Transmitter;
    i2c_comm_state  = COMM_PRE;
    /** initialize static parameter according to input parameter */
    SlaveADDR    = EEPROM_ADDRESS; /// this byte shoule be send by F/W (in loop or INTSTS way)
    DeviceOffset = WriteAddr;      /// this byte can be send by both F/W and DMA
    OffsetDone   = FALSE;
    memcpy(SendBuf, pBuffer, NumByteToWrite);
    BufCount           = 0;
    Int_NumByteToWrite = NumByteToWrite;
    I2C_Acknowledg_Enable(I2Cx);
    I2C_Interrupts_Enable(I2Cx, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR);

    /** Send START condition */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_Flag_Status_Get(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    I2C_Generate_Start_Enable(I2Cx);
    I2C_EE_WaitOperationIsCompleted();
    I2C_EE_WriteOnePageCompleted();
#endif
}

/**
*\*\name    I2C_EE_WriteOnePageCompleted.
*\*\fun     Process Write one page completed.
*\*\param   none
*\*\return  none 
**/
void I2C_EE_WriteOnePageCompleted(void)
{
    I2C_pBuffer += I2C_NumByteWritingNow;
    I2C_WriteAddr += I2C_NumByteWritingNow;
    I2C_NumByteToWrite -= I2C_NumByteWritingNow;
}

/**
*\*\name  I2C_EE_ReadBuffer.
*\*\fun   Reads a block of data from the EEPROM.
*\*\param pBuffer pointer to the buffer that receives the data read
*\*\                  from the EEPROM.
*\*\param ReadAddr EEPROM's internal address to read from.
*\*\param NumByteToRead number of bytes to read from the EEPROM.
**/
void I2C_EE_ReadBuffer(u8* pBuffer, u16 ReadAddr, u16 NumByteToRead)
{
#if PROCESS_MODE == 0 /* polling */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_Flag_Status_Get(I2Cx, I2C_FLAG_BUSY))
    {
            if ((sEETimeout--) == 0)
                    sEE_TIMEOUT_UserCallback();
    }
    
    I2C_PEC_Position_Set(I2Cx, I2C_PEC_POS_CURRENT);
    I2C_Acknowledg_Enable(I2Cx);
    
    /** Send START condition */
    I2C_Generate_Start_Enable(I2Cx);

    /** Test on EV5 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send EEPROM address for write */
    I2C_7bit_Addr_Send(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_SEND);
    /** Test on EV6 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_TXMODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Clear EV6 by setting again the PE bit */
    I2C_ON(I2Cx);
    /** Send the EEPROM's internal address to write to */
    I2C_Data_Send(I2Cx, ReadAddr);
    /** Test on EV8 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_DATA_SENDED))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send STRAT condition a second time */
    I2C_Generate_Start_Enable(I2Cx);
    /** Test on EV5 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    /** Send EEPROM address for read */
    I2C_7bit_Addr_Send(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_RECV);
    /* Test on EV6 and clear it */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (!I2C_Flag_Status_Get(I2Cx, I2C_FLAG_ADDRF))    //EV6
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }

    /** While there is data to be read */
    if (NumByteToRead == 1)
    {
        /** Disable Acknowledgement */
        I2C_Acknowledg_Disable(I2Cx);
        /** clear ADDR */
        (void)(I2Cx->STS1); 
        (void)(I2Cx->STS2);
        /** Generates START condition to close communication */
        I2C_Generate_Start_Enable(I2Cx);
    }
    else
    {
        /** clear ADDR */
        (void)(I2Cx->STS1); 
        (void)(I2Cx->STS2);
    }
    
    while (NumByteToRead)
    {
        if (NumByteToRead <= 2)
        {
            /** One byte */
            if (NumByteToRead == 1)
            {
                /** Wait until RXNE flag is set */
                sEETimeout = sEE_LONG_TIMEOUT;
                while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_DATA_RECVD_FLAG))
                {
                    if ((sEETimeout--) == 0)
                        sEE_TIMEOUT_UserCallback();
                }
                /** Read data from DAT */
                *pBuffer = I2C_Data_Recv(I2Cx);
                /** Point to the next location where the byte read will be saved */
                pBuffer++;
                /** Decrement the read bytes counter */
                NumByteToRead--;
                 /** Generates STOP condition to release SCL/SDA line */
                I2C_Generate_Stop_Enable(I2Cx);   
             
            }
            /** 2 Last bytes */
            else
            {   
                /** Wait until RXNE flag is set */
                sEETimeout = sEE_LONG_TIMEOUT;
                while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_DATA_RECVD_FLAG))
                {
                    if ((sEETimeout--) == 0)
                        sEE_TIMEOUT_UserCallback();
                }
                /** Disable Acknowledgement */ 
                I2C_Acknowledg_Disable(I2Cx);
                               
                /** Read data from DAT */
                *pBuffer = I2C_Data_Recv(I2Cx);
                /** Point to the next location where the byte read will be saved */
                pBuffer++;
                /** Decrement the read bytes counter */
                NumByteToRead--;
                
                /** Generates START condition to close communication */
                I2C_Generate_Start_Enable(I2Cx);
                
                while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_DATA_RECVD_FLAG))
                {
                    if ((sEETimeout--) == 0)
                        sEE_TIMEOUT_UserCallback();
                }                                                 
                /** Read data from DAT */
                *pBuffer = I2C_Data_Recv(I2Cx);              
                /** Point to the next location where the byte read will be saved */
                pBuffer++;
                /** Decrement the read bytes counter */
                NumByteToRead--;
                
                /** Generates STOP condition to release SCL/SDA line */
                I2C_Generate_Stop_Enable(I2Cx);
            }
        }
        else
        {
            /** Test on EV7 and clear it */
            sEETimeout = sEE_LONG_TIMEOUT;
            while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_DATA_RECVD_FLAG))
            {
                if ((sEETimeout--) == 0)
                    sEE_TIMEOUT_UserCallback();
            }
            /** Read a byte from the EEPROM */
            *pBuffer = I2C_Data_Recv(I2Cx);
            /** Point to the next location where the byte read will be saved */
            pBuffer++;
            /** Decrement the read bytes counter */
            NumByteToRead--;
            if (I2C_Flag_Status_Get(I2Cx, I2C_FLAG_BYTEF))
            {
                /** Read a byte from the EEPROM */
                *pBuffer = I2C_Data_Recv(I2Cx);
                /** Point to the next location where the byte read will be saved */
                pBuffer++;
                /** Decrement the read bytes counter */
                NumByteToRead--;
            }
        }
    }
#elif PROCESS_MODE == 1 /* interrupt */
    I2C_pBuffer     = pBuffer;
    MasterDirection = Receiver;

    /** initialize static parameter according to input parameter*/
    SlaveADDR      = EEPROM_ADDRESS;
    DeviceOffset   = ReadAddr;
    OffsetDone     = FALSE;
    i2c_comm_state = COMM_PRE;
    I2C_Acknowledg_Enable(I2Cx);
    I2C_Interrupts_Enable(I2Cx, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR);
    Int_NumByteToRead = NumByteToRead;

    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_Flag_Status_Get(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
    I2C_Generate_Start_Enable(I2Cx);
    I2C_EE_WaitOperationIsCompleted();
#endif    
}

/**
*\*\name    I2C_EE_WaitOperationIsCompleted.
*\*\fun     wait operation is completed.
*\*\param   none
*\*\return  none 
**/
void I2C_EE_WaitOperationIsCompleted(void)
{
    sEETimeout = sEE_LONG_TIMEOUT;
    while (i2c_comm_state != COMM_DONE)
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }
}

/**
*\*\name    I2C1_EV_IRQHandler.
*\*\fun     I2c1 event interrupt Service Routines.
*\*\param   none
*\*\return  none 
**/
void I2C1_EV_IRQHandler(void)
{
    uint32_t lastevent = I2C_Last_Event_Get(I2C1);
    switch (lastevent)
    {
    /** Master Invoke */
    case I2C_EVT_MASTER_MODE_FLAG: /// EV5
        if (!check_begin)
        {
            i2c_comm_state = COMM_IN_PROCESS;
        }
        if (MasterDirection == Receiver)
        {
            if (!OffsetDone)
            {
                I2C_7bit_Addr_Send(I2C1, SlaveADDR, I2C_DIRECTION_SEND);
            }
            else
            {
                /** Send slave Address for read */
                I2C_7bit_Addr_Send(I2C1, SlaveADDR, I2C_DIRECTION_RECV);
                OffsetDone = FALSE;
            }
        }
        else
        {
            /** Send slave Address for write */
            I2C_7bit_Addr_Send(I2C1, SlaveADDR, I2C_DIRECTION_SEND);
        }
        break;
    /** Master Receiver events */
    case I2C_EVT_MASTER_RXMODE_FLAG: /// EV6
        break;
    case I2C_EVT_MASTER_DATA_RECVD_FLAG: /// EV7
        *I2C_pBuffer = I2C_Data_Recv(I2C1);
        I2C_pBuffer++;
        Int_NumByteToRead--;
        if (Int_NumByteToRead == 1)
        {
            /** Disable Acknowledgement */
            I2C_Acknowledg_Disable(I2C1);
            I2C_Generate_Stop_Enable(I2C1);
        }
        if (Int_NumByteToRead == 0)
        {
            I2C_Interrupts_Disable(I2C1, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR);
            i2c_comm_state = COMM_DONE;
        }
        break;
    /** Master Transmitter events */
    case I2C_EVT_MASTER_TXMODE_FLAG: /// EV8 just after EV6
        if (check_begin)
        {
            check_begin = FALSE;
            I2C_Interrupts_Disable(I2C1, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR);
            I2C_Generate_Stop_Enable(I2C1);
            i2c_comm_state = COMM_DONE;
            break;
        }
        I2C_Data_Send(I2C1, DeviceOffset);
        OffsetDone = TRUE;
        break;
    case I2C_EVT_MASTER_DATA_SENDING: ///  EV8 I2C_EVENT_MASTER_DATA_TRANSMITTING
        if (MasterDirection == Receiver)
        {
            I2C_Generate_Start_Enable(I2C1);
        }
        break;
    case I2C_EVT_MASTER_DATA_SENDED: /// EV8-2
        if (MasterDirection == Transmitter)
        {
            if (Int_NumByteToWrite == 0)
            {
                I2C_Generate_Stop_Enable(I2C1);
                sEETimeout = sEE_LONG_TIMEOUT;
                while (I2C_Flag_Status_Get(I2Cx, I2C_FLAG_BUSY))
                {
                    if ((sEETimeout--) == 0)
                        sEE_TIMEOUT_UserCallback();
                }
                check_begin = TRUE;
                I2C_Generate_Start_Enable(I2C1);
            }
            else
            {
                I2C_Data_Send(I2C1, SendBuf[BufCount++]);
                Int_NumByteToWrite--;
            }
        }
        break;
    }
}

/**
*\*\name    I2C1_ER_IRQHandler.
*\*\fun     I2c1 error interrupt Service Routines.
*\*\param   none
*\*\return  none 
**/
void I2C1_ER_IRQHandler(void)
{
    if (I2C_Flag_Status_Get(I2C1, I2C_FLAG_ACKFAIL))
    {
        if (check_begin) /// EEPROM write busy
        {
            I2C_Generate_Start_Enable(I2C1);
        }
        else if (I2C1->STS2 & 0x01) /// real fail
        {
            I2C_Generate_Stop_Enable(I2C1);
            i2c_comm_state = COMM_EXIT;
        }
        I2C_Flag_Status_Clear(I2C1, I2C_FLAG_ACKFAIL);
    }

    if (I2C_Flag_Status_Get(I2C1, I2C_FLAG_BUSERR))
    {
        if (I2C1->STS2 & 0x01)
        {
            I2C_Generate_Stop_Enable(I2C1);
            i2c_comm_state = COMM_EXIT;
        }
        I2C_Flag_Status_Clear(I2C1, I2C_FLAG_BUSERR);
    }
}

/**
*\*\name    I2C_EE_WaitEepromStandbyState.
*\*\fun     Wait eeprom standby state.
*\*\param   none
*\*\return  none 
**/
void I2C_EE_WaitEepromStandbyState(void)
{
    __IO uint16_t tmpSR1    = 0;
    __IO uint32_t sEETrials = 0;

    /** While the bus is busy */
    sEETimeout = sEE_LONG_TIMEOUT;
    while (I2C_Flag_Status_Get(I2Cx, I2C_FLAG_BUSY))
    {
        if ((sEETimeout--) == 0)
            sEE_TIMEOUT_UserCallback();
    }

    /** Keep looping till the slave acknowledge his address or maximum number
       of trials is reached (this number is defined by sEE_MAX_TRIALS_NUMBER) */
    while (1)
    {
        /** Send START condition */
        I2C_Generate_Start_Enable(I2Cx);

        /** Test on EV5 and clear it */
        sEETimeout = sEE_LONG_TIMEOUT;
        while (!I2C_Event_Check(I2Cx, I2C_EVT_MASTER_MODE_FLAG))
        {
            if ((sEETimeout--) == 0)
                sEE_TIMEOUT_UserCallback();
        }

        /** Send EEPROM address for write */
        I2C_7bit_Addr_Send(I2Cx, EEPROM_ADDRESS, I2C_DIRECTION_SEND);
        /** Wait for ADDR flag to be set (Slave acknowledged his address) */
        sEETimeout = sEE_LONG_TIMEOUT;
        do
        {
            /** Get the current value of the STS1 register */
            tmpSR1 = I2Cx->STS1;

            /** Update the timeout value and exit if it reach 0 */
            if ((sEETimeout--) == 0)
                sEE_TIMEOUT_UserCallback();
        }
        /** Keep looping till the Address is acknowledged or the AF flag is
           set (address not acknowledged at time) */
        while ((tmpSR1 & (I2C_STS1_ADDRF | I2C_STS1_ACKFAIL)) == 0);

        /** Check if the ADDR flag has been set */
        if (tmpSR1 & I2C_STS1_ADDRF)
        {
            /** Clear ADDR Flag by reading STS1 then STS2 registers (STS1 have already
               been read) */
            (void)I2Cx->STS2;

            /** STOP condition */
            I2C_Generate_Stop_Enable(I2Cx);

            /** Exit the function */
            return;
        }
        else
        {
            /** Clear AF flag */
            I2C_Flag_Status_Clear(I2Cx, I2C_FLAG_ACKFAIL);
        }

        /** Check if the maximum allowed numbe of trials has bee reached */
        if (sEETrials++ == sEE_MAX_TRIALS_NUMBER)
        {
            /** If the maximum number of trials has been reached, exit the function */
            sEE_TIMEOUT_UserCallback();
        }
    }
}


