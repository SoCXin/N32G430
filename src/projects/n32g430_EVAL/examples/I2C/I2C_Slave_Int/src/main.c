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

/** I2C_Slave_Int **/



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
static __IO uint32_t I2CTimeout;
uint8_t flag_slave_recv_finish         = 0;
uint8_t flag_slave_send_finish         = 0;
static uint8_t rxDataNum = 0;
static uint8_t RCC_RESET_Flag = 0;

void CommTimeOut_CallBack(ErrCode_t errcode);

/**
*\*\name    Delay.
*\*\fun     system ms delay function.
*\*\param   nCount
*\*\return  none 
**/
void Delay(uint32_t nCount)
{
    uint32_t tcnt;
    while (nCount--)
    {
        tcnt = 48000 / 5;
        while (tcnt--){;}
    }
}

/**
*\*\name    NVIC_ConfigurationSlave.
*\*\fun     NVIC Configuration.
*\*\param   ch:1-i2c1 2-i2c2
*\*\return  result 
**/
void NVIC_ConfigurationSlave(uint8_t ch)
{
    NVIC_InitType NVIC_InitStructure;
    if (ch == 1)
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
    }
    if (ch == 2)
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
    }

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
    if (ch == 1)
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn; /* test err */
    }
    if (ch == 2)
    {
        NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn; /* test err */
    }
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}



/**
*\*\name    i2c_slave_init.
*\*\fun     slave gpio/rcc/i2c/nvic initializes.
*\*\param   none
*\*\return  result 
**/
int i2c_slave_init(void)
{
    I2C_InitType i2c1_slave;
    GPIO_InitType i2c1_gpio;
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
    i2c1_slave.ClkSpeed    = 100000; /* 100K */

    I2C_Initializes(I2Cx, &i2c1_slave);
    /* int enable */
    I2C_Interrupts_Enable(I2Cx, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR);
    NVIC_ConfigurationSlave((uint8_t)1);
    I2C_ON(I2Cx);
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
    log_info(" this is a i2c slave int demo\r\n");

    /* Initialize the I2C slave driver ----------------------------------------*/
    i2c_slave_init();
    
    I2CTimeout = I2CT_LONG_TIMEOUT * 1000;
    /* recive data */
    while (flag_slave_recv_finish == 0) /* wait for recv data finish */
    {
        if ((I2CTimeout--) == 0)
            CommTimeOut_CallBack(SLAVE_UNKNOW);
    }
        
    log_info(" recv finish,recv len = %d\r\n", rxDataNum);
    flag_slave_recv_finish = 0;

    I2CTimeout = I2CT_LONG_TIMEOUT * 1000;
    /* send data */
    while (flag_slave_send_finish == 0) /* wait for send data finish */
    {
        if ((I2CTimeout--) == 0)
            CommTimeOut_CallBack(SLAVE_UNKNOW);
    }
        
    log_info(" tx finish,tx len = %d\r\n", rxDataNum);
    flag_slave_send_finish = 0;
        
    while (1)
    {;}
}


/**
*\*\name    I2C1_EV_IRQHandler.
*\*\fun     i2c Interrupt service function.
*\*\param   none
*\*\return  none 
**/
void I2C1_EV_IRQHandler(void)
{
    uint8_t timeout_flag = 0;
    uint32_t last_event = 0;
    
    last_event = I2C_Last_Event_Get(I2C1);
    if ((last_event & I2C_ROLE_MASTER) != I2C_ROLE_MASTER) /* MSMODE = 0:I2C slave mode */
    {
        switch (last_event)
        {
        case I2C_EVT_SLAVE_RECV_ADDR_MATCHED: /*0x00020002.EV1 Rx addr matched */
            
            /* clear flag,ready to receive data */
            rxDataNum = 0;
            break;
        
        case I2C_EVT_SLAVE_SEND_ADDR_MATCHED: /*0x00060082.EV1 Tx addr matched */
            
            rxDataNum = 0;
            I2C1->DAT = data_buf[rxDataNum++];/* Send first data */
            break;
        
        /* SlaveTransmitter	 */
        case I2C_EVT_SLAVE_DATA_SENDING:  /*0x00060080. EV3 Sending data */
           
            break;
            
        case I2C_EVT_SLAVE_DATA_SENDED:
            I2C1->DAT = data_buf[rxDataNum++];
            break;
        
        /* SlaveReceiver */
        case I2C_EVT_SLAVE_DATA_RECVD: /*0x00020040.EV2 one byte recved */
            data_buf[rxDataNum++] = I2C1->DAT;
            break;
       
        
        case I2C_EVT_SLAVE_STOP_RECVD: /* 0x00000010 EV4 */
            I2C_ON(I2C1);   
            if(rxDataNum != 0)
            {
                flag_slave_recv_finish = 1; /* The STOPF bit is not set after a NACK reception */
            }
            break;
        
        default:
            I2C_ON(I2C1);
            timeout_flag = 1;
            break;
        }
    }
    
    if (timeout_flag)
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(SLAVE_UNKNOW);
        }
    }
    else
    {
        I2CTimeout = I2CT_LONG_TIMEOUT;
    }
       
}

/**
*\*\name    I2C1_ER_IRQHandler.
*\*\fun     i2c error interrupt service function.
*\*\param   none
*\*\return  none 
**/
void I2C1_ER_IRQHandler(void)
{
    uint32_t last_event;
    last_event = I2C_Last_Event_Get(I2C1);
    if(last_event == I2C_EVT_SLAVE_ACK_MISS)   
    {   
        I2C_Flag_Status_Clear(I2C1, I2C_FLAG_ACKFAIL);
        if(rxDataNum != 0)  /*slave send the last data and recv NACK  */
        {
            flag_slave_send_finish = 1;
        }
        else /*not the last data recv nack, send fail */
        {
        }
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

