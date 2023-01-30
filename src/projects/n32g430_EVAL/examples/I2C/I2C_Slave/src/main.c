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
#include "n32g430_i2c.h"
#include "main.h"
#include "log.h"

/** I2C_Slave **/


#define I2C_SLAVE_LOW_LEVEL
#define TEST_BUFFER_SIZE  100
#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
#define I2C_SLAVE_ADDR    0x10

#define I2C1_TEST
#define I2Cx I2C1
#define I2Cx_SCL_PIN GPIO_PIN_6
#define I2Cx_SDA_PIN GPIO_PIN_7
#define GPIOx        GPIOB
#define GPIO_AF_I2C  GPIO_AF2_I2C1

uint8_t data_buf[TEST_BUFFER_SIZE] = {0};
volatile Status test_status        = FAILED;

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

static __IO uint32_t I2CTimeout = I2CT_LONG_TIMEOUT;
static uint8_t RCC_RESET_Flag = 0;

void CommTimeOut_CallBack(ErrCode_t errcode);

/**
*\*\name    i2c_slave_init.
*\*\fun     slave gpio/rcc/i2c initializes.
*\*\param   none
*\*\return  result 
**/
int i2c_slave_init(void)
{
    I2C_InitType i2c1_slave;
    GPIO_InitType i2c1_gpio;
    /* enable clk */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_I2C1);
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);

    GPIO_Structure_Initialize(&i2c1_gpio);
    /*PB6 -- SCL; PB7 -- SDA*/
    i2c1_gpio.Pin        = I2Cx_SCL_PIN | I2Cx_SDA_PIN;
    i2c1_gpio.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST; 
    i2c1_gpio.GPIO_Mode      = GPIO_MODE_AF_OD;
    i2c1_gpio.GPIO_Alternate = GPIO_AF_I2C;
    GPIO_Peripheral_Initialize(GPIOx, &i2c1_gpio);

    I2C_Reset(I2Cx);
    I2C_Initializes_Structure(&i2c1_slave);
    i2c1_slave.BusMode     = I2C_BUSMODE_I2C;
    i2c1_slave.DutyCycle   = I2C_SMDUTYCYCLE_1;  
    i2c1_slave.OwnAddr1    = I2C_SLAVE_ADDR;
    i2c1_slave.AckEnable   = I2C_ACKEN;
    i2c1_slave.AddrMode    = I2C_ADDR_MODE_7BIT;
    i2c1_slave.ClkSpeed    = 100000; /* 100k */

    I2C_Initializes(I2Cx, &i2c1_slave);

    I2C_ON(I2Cx);
    return 0;
}

/**
*\*\name    i2c_slave_send.
*\*\fun     slave send data.
*\*\param   data-data to send
*\*\param   len-length of data to send
*\*\return  send result 
**/
int i2c_slave_send(uint8_t* data, int len)
{
    uint32_t lastevent;
    uint8_t* sendBufferPtr = data;
    I2CTimeout = I2CT_LONG_TIMEOUT;
    
    while (!I2C_Event_Check(I2C1, I2C_EVT_SLAVE_SEND_ADDR_MATCHED))
        ; /* send addr matched */

    I2C_ON(I2C1);
    I2C_Data_Send(I2C1, *sendBufferPtr++);
    while (1)
    {
        lastevent = I2C_Last_Event_Get(I2C1);
        if (lastevent == I2C_EVT_SLAVE_DATA_SENDED)
        {
            I2C_Data_Send(I2C1, *sendBufferPtr++);
            I2CTimeout = I2CT_LONG_TIMEOUT;
        }
        else if ((lastevent == I2C_EVT_SLAVE_ACK_MISS) || (lastevent == I2C_EVT_SLAVE_STOP_RECVD))
        {
            I2C_Flag_Status_Clear(I2C1, I2C_FLAG_ACKFAIL);
            I2C_ON(I2C1);
            break;
        }
        else if ((lastevent & 0x00000100) == 0x00000100)
        {
            CommTimeOut_CallBack(SLAVE_BUSERR);
        }
        else
        {
            if ((I2CTimeout--) == 0)
                CommTimeOut_CallBack(SLAVE_UNKNOW);
        }
    }
    return 0;
}

/**
*\*\name    i2c_slave_recv.
*\*\fun     slave receive data.
*\*\param   data-data to receive
*\*\param   rcv_len-length of data to receive
*\*\return  receive result
**/
int i2c_slave_recv(uint8_t* data, uint32_t rcv_len)
{
    uint32_t lastevent;
    I2CTimeout = I2CT_LONG_TIMEOUT;
    
    while (!I2C_Event_Check(I2C1, I2C_EVT_SLAVE_RECV_ADDR_MATCHED))
        ; /* receive addr matched */

    I2C_ON(I2C1);
    while (1)
    {
        lastevent = I2C_Last_Event_Get(I2C1);
        if (lastevent == I2C_EVT_SLAVE_DATA_RECVD)
        {
            *data++ = I2C_Data_Recv(I2C1);
            I2CTimeout = I2CT_LONG_TIMEOUT;
        }
        else if (lastevent == I2C_EVT_SLAVE_STOP_RECVD)
        {
            I2C_ON(I2C1);
            break;
        }
        else if ((lastevent & 0x00000100) == 0x00000100)
        {
            CommTimeOut_CallBack(SLAVE_BUSERR);
        }
        else
        {
            if ((I2CTimeout--) == 0)
                CommTimeOut_CallBack(SLAVE_UNKNOW);
        }
    }
    
    return 0;
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
    log_info("this is a i2c slave test demo\r\n");
    /* Initialize the I2C EEPROM driver ----------------------------------------*/
    i2c_slave_init();

    /* Read data */
    log_info("i2c slave recv data start\r\n");
    i2c_slave_recv(data_buf, TEST_BUFFER_SIZE);

    /* Write data*/
    log_info("i2c slave send data start\r\n");
    if (0 == i2c_slave_send(data_buf, TEST_BUFFER_SIZE))
        log_info("i2c slave test pass\r\n");
    else
        log_info("i2c slave test fail\r\n");
    
    while (1)
    {
    }
}

/**
*\*\name    SystemNVICReset.
*\*\fun     System software reset.
*\*\param   none
*\*\return  none 
**/    
void SystemNVICReset(void)
{
    
    __disable_irq();
    log_info("***** NVIC system reset! *****\r\n");
    NVIC_SystemReset();
}

/**
*\*\name    IIC_RCCReset.
*\*\fun     RCC clock reset.
*\*\param   none
*\*\return  none 
**/
void IIC_RCCReset(void)
{
    if (RCC_RESET_Flag >= 3)
    {
        SystemNVICReset();
    }
    else
    {
        RCC_RESET_Flag++;
        
        RCC_APB1_Peripheral_Reset(RCC_APB1_PERIPH_I2C1);
        
        RCC_APB1_Peripheral_Clock_Disable(RCC_APB1_PERIPH_I2C1);
        GPIOB->PMODE &= 0xFFFF0FFF; /*input */
        RCC_APB2_Peripheral_Clock_Disable(RCC_APB2_PERIPH_AFIO);
        RCC_AHB_Peripheral_Clock_Disable(RCC_AHB_PERIPH_GPIOB);
        
        RCC_APB1_Peripheral_Reset(RCC_APB1_PERIPH_I2C1);
        
        
        log_info("***** IIC module by RCC reset! *****\r\n");
        i2c_slave_init();
    }
}

/**
*\*\name    IIC_SWReset.
*\*\fun     I2c software reset.
*\*\param   none
*\*\return  none 
**/
void IIC_SWReset(void)
{
    GPIO_InitType i2cx_gpio;
    
    GPIO_Structure_Initialize(&i2cx_gpio);
    i2cx_gpio.Pin            = I2Cx_SCL_PIN | I2Cx_SDA_PIN;
    i2cx_gpio.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    i2cx_gpio.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_Peripheral_Initialize(GPIOx, &i2cx_gpio);
    
    I2CTimeout = I2CT_LONG_TIMEOUT;
    for (;;)
    {
        if ((I2Cx_SCL_PIN | I2Cx_SDA_PIN) == (GPIOx->PID & (I2Cx_SCL_PIN | I2Cx_SDA_PIN)))
        {
            I2Cx->CTRL1 |= 0x8000;
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            I2Cx->CTRL1 &= ~0x8000;
            
            log_info("***** IIC module self reset! *****\r\n");
            break;
        }
        else
        {
            if ((I2CTimeout--) == 0)
            {
                IIC_RCCReset();
            }
        }
    }
}

/**
*\*\name    CommTimeOut_CallBack.
*\*\fun     Callback function.
*\*\param   none
*\*\return  none 
**/
void CommTimeOut_CallBack(ErrCode_t errcode)
{
    log_info("...ErrCode:%d\r\n", errcode);
    
#if (COMM_RECOVER_MODE == MODULE_SELF_RESET)
    IIC_SWReset();
#elif (COMM_RECOVER_MODE == MODULE_RCC_RESET)
    IIC_RCCReset();
#elif (COMM_RECOVER_MODE == SYSTEM_NVIC_RESET)
    SystemNVICReset();
#endif
}


