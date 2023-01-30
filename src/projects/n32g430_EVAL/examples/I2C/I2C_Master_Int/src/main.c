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
*\*\version v1.0.1
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "n32g430.h"
#include "n32g430_i2c.h"
#include "main.h"
#include "log.h"


/** I2C_Master_Int **/

#define I2C_MASTER_LOW_LEVEL
#define TEST_BUFFER_SIZE  100
#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))
#define I2C_MASTER_ADDR   0x30
#define I2C_SLAVE_ADDR    0x10

#define I2C1_TEST
#define I2Cx I2C1
#define I2Cx_SCL_PIN GPIO_PIN_6
#define I2Cx_SDA_PIN GPIO_PIN_7
#define GPIOx        GPIOB
#define GPIO_AF_I2C GPIO_AF2_I2C1

static uint8_t tx_buf[TEST_BUFFER_SIZE] = {0};
static uint8_t rx_buf[TEST_BUFFER_SIZE] = {0};
volatile Status test_status      = FAILED;
static __IO uint32_t I2CTimeout;
static CommCtrl_t Comm_Flag = C_READY; /* Avoid star flag and stop flag set 1 at the same time */
static uint8_t RCC_RESET_Flag = 0;

static uint8_t rx_num = 0;
static uint8_t tx_num = 0;
uint8_t flag_master_recv_finish = 0;
uint8_t flag_master_send_finish = 0;
uint8_t flag_trans_direct       = 0; /* write */

Status Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void Memset(void* s, uint8_t c, uint32_t count);
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
*\*\name    Delay_us.
*\*\fun     system us delay function.
*\*\param   nCount
*\*\return  none 
**/
void Delay_us(uint32_t nCount)
{
    uint32_t tcnt;
    while (nCount--)
    {
        tcnt = 48 / 5;
        while (tcnt--){;}
    }
}

/**
*\*\name    NVIC_ConfigurationMaster.
*\*\fun     NVIC Configuration.
*\*\param   ch:1-i2c1 2-i2c2
*\*\return  result 
**/
void NVIC_ConfigurationMaster(uint8_t ch)
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
*\*\name    i2c_master_init.
*\*\fun     master gpio/rcc/i2c/nvic initializes.
*\*\param   none
*\*\return  result 
**/
int i2c_master_init(void)
{
    I2C_InitType i2c1_master;
    GPIO_InitType i2c1_gpio;
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_I2C1);
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);

    GPIO_Structure_Initialize(&i2c1_gpio);
    /*PB6 -- SCL; PB7 -- SDA*/
    i2c1_gpio.Pin            = I2Cx_SCL_PIN | I2Cx_SDA_PIN;
    i2c1_gpio.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST; 
    i2c1_gpio.GPIO_Mode      = GPIO_MODE_AF_OD;
    i2c1_gpio.GPIO_Alternate = GPIO_AF_I2C;
    i2c1_gpio.GPIO_Pull      = GPIO_PULL_UP; 
    GPIO_Peripheral_Initialize(GPIOx, &i2c1_gpio);

    I2C_Reset(I2Cx);
    I2C_Initializes_Structure(&i2c1_master);
    i2c1_master.BusMode     = I2C_BUSMODE_I2C;
    i2c1_master.DutyCycle   = I2C_SMDUTYCYCLE_1;
    i2c1_master.OwnAddr1    = I2C_MASTER_ADDR;
    i2c1_master.AckEnable   = I2C_ACKEN;
    i2c1_master.AddrMode    = I2C_ADDR_MODE_7BIT;
    i2c1_master.ClkSpeed    = 100000; /* 100K */

    I2C_Initializes(I2Cx, &i2c1_master);
    /* int enable */
    I2C_Interrupts_Enable(I2Cx, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR);
    NVIC_ConfigurationMaster(1);
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
    uint16_t i = 0;
    
    log_init();
    log_info("\n\rthis is a i2c master int demo\r\n");

    i2c_master_init();

    /* Fill the buffer to send */
    for (i = 0; i < TEST_BUFFER_SIZE; i++)
    {
        tx_buf[i] = i;
    }
    /* First write in the memory followed by a read of the written data --------*/
    /* Write data*/
    flag_trans_direct = 0;
    if (Comm_Flag == C_READY)
    {
        Comm_Flag = C_START_BIT;
        I2C_Generate_Start_Enable(I2C1);
    }
    
    I2CTimeout = I2CT_LONG_TIMEOUT * 1000;
    while (flag_master_send_finish == 0)
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_UNKNOW);
        }
    }
    
    /* master send finish */
    flag_master_send_finish = 0;
    I2CTimeout = I2CT_LONG_TIMEOUT;
    while (I2C_Flag_Status_Get(I2C1, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_BUSY);
        }
    }
    Comm_Flag = C_READY;

    I2C_Acknowledg_Enable(I2C1);
    /* read data */
    flag_trans_direct = 1;
    if (Comm_Flag == C_READY)
    {
        Comm_Flag = C_START_BIT;
        I2C_Generate_Start_Enable(I2C1);
    }
    
    I2CTimeout = I2CT_LONG_TIMEOUT * 1000;
    while (flag_master_recv_finish == 0)
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_UNKNOW);
        }
    }
        
    /* master recv finish */
    flag_master_recv_finish = 0;
    I2CTimeout = I2CT_LONG_TIMEOUT;
    while (I2C_Flag_Status_Get(I2C1, I2C_FLAG_BUSY))
    {
        if ((I2CTimeout--) == 0)
        {
            CommTimeOut_CallBack(MASTER_BUSY);
        }
    }
    Comm_Flag = C_READY;

    /* Check if the data written to the memory is read correctly */
    test_status = Buffercmp(tx_buf, rx_buf, TEST_BUFFER_SIZE);
    if (test_status == PASSED) /* test_status = PASSED, if the write and read dataare the same  */
    {
        log_info("the write and read data the same, i2c master int test pass\r\n");
    }
    else /* test_status = FAILED, if the write and read dataare different */
    {
        log_info("the write and read data are different, i2c master int test fail\r\n");
    }

    while (1)
    {
    }
}

/**
*\*\name    I2C1_EV_IRQHandler.
*\*\fun     i2c event Interrupt service function.
*\*\param   none
*\*\return  none 
**/
void I2C1_EV_IRQHandler(void)
{
    uint32_t last_event = 0;

    last_event = I2C_Last_Event_Get(I2C1);
    if ((last_event & I2C_ROLE_MASTER) == I2C_ROLE_MASTER) /* master mode */
    {
        switch (last_event)
        {
        case I2C_EVT_MASTER_MODE_FLAG: /* 0x00030001.EV5 Send addr */
            
            if(flag_trans_direct)     /* read */
            {
                Memset(rx_buf, 0, TEST_BUFFER_SIZE); /* clear recv buf, ready to recv data */
                I2C_7bit_Addr_Send(I2C1, I2C_SLAVE_ADDR, I2C_DIRECTION_RECV);
                rx_num = 0;
            }
            else /* write */
            {
                I2C_7bit_Addr_Send(I2C1, I2C_SLAVE_ADDR, I2C_DIRECTION_SEND);
                tx_num = 0;
            }
            break;

           /* MasterTransmitter	 */
        case I2C_EVT_MASTER_TXMODE_FLAG: /* 0x00070082. EV6 Send first data */
            Comm_Flag = C_READY;
            I2C_Data_Send(I2C1, tx_buf[tx_num++]);
            break;

        case I2C_EVT_MASTER_DATA_SENDING: /* 0x00070080. EV8 Sending data  */
            if (tx_num < TEST_BUFFER_SIZE)
            {
                I2C_Data_Send(I2C1, tx_buf[tx_num++]);
            }
            break;
            
        case I2C_EVT_MASTER_DATA_SENDED: /* 0x00070084.EV8_2 Send finish */
            if (tx_num == TEST_BUFFER_SIZE)
            {
                if (Comm_Flag == C_READY)
                {
                    Comm_Flag = C_STOP_BIT;
                    I2C_Generate_Stop_Enable(I2C1);
                    flag_master_send_finish = 1;
                }
            }
            break;
            
            /* MasterReceiver */
        case I2C_EVT_MASTER_RXMODE_FLAG: /* 0x00030002.EV6 */
            Comm_Flag = C_READY;
            if (TEST_BUFFER_SIZE == 1)
            {
                I2C_Acknowledg_Disable(I2C1);
                if (Comm_Flag == C_READY)
                {
                    Comm_Flag = C_STOP_BIT;
                    I2C_Generate_Stop_Enable(I2C1);
                }
            }
            else if (TEST_BUFFER_SIZE == 2)
            {
                I2C1->CTRL1 |= I2C_NACK_POS_NEXT; /* set ACKPOS */
                I2C_Acknowledg_Disable(I2C1);
            }
            break;

        case I2C_EVT_MASTER_DATA_RECVD_FLAG: /* 0x00030040. EV7.one byte recved  */
        case I2C_EVT_MASTER_DATA_RECVD_BSF_FLAG: /* 0x00030044. EV7.When the I2C communication rate is too high, BSF = 1 */
            
            rx_buf[rx_num++] = I2C_Data_Recv(I2C1);
            if (rx_num == (TEST_BUFFER_SIZE - 1))
            { 
                I2C_Acknowledg_Disable(I2C1);   /* Disable I2C1 acknowledgement. */
                if (Comm_Flag == C_READY)
                {
                    Comm_Flag = C_STOP_BIT;
                    I2C_Generate_Stop_Enable(I2C1); /* Send I2C1 STOP Condition. */
                }
            }
            else if (rx_num == TEST_BUFFER_SIZE)
            {
                flag_master_recv_finish = 1;
            }
            break;
        case 0x00030201: /* Arbitration lost */
        case 0x00030401: /* Acknowledge failure */
        case 0x00030501: /* Acknowledge failure and Bus error */
        case 0x00070000: /* TRF, BUSY, MSMODE flags */
            I2C_OFF(I2C1);
            I2C_ON(I2C1);
            break;

        default:
            log_info("I2C error status:0x%x\r\n", last_event);
            break;
        }
    }
}

/**
*\*\name    I2C1_ER_IRQHandler.
*\*\fun     i2c error Interrupt service function.
*\*\param   none
*\*\return  none 
**/
void I2C1_ER_IRQHandler(void)
{
		if(I2C_Flag_Status_Get(I2C1, I2C_FLAG_ACKFAIL))
		{
				I2C_Flag_Status_Clear(I2C1, I2C_FLAG_ACKFAIL);
				I2C_Generate_Stop_Enable(I2C1); /* Send I2C1 STOP Condition. */
		}
}

/**
*\*\name   Buffercmp.
*\*\fun    Compares two buffers.
*\*\param  pBuffer, pBuffer1: buffers to be compared.
*\*\param  BufferLength: buffer's length
*\*\return PASSED: pBuffer identical to pBuffer1
*\*\       FAILED: pBuffer differs from pBuffer1
**/
Status Buffercmp(uint8_t* pBuffer, uint8_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}

/**
*\*\name  Memset.
*\*\fun   memery set a value
*\*\param s source
*\*\param c value
*\*\param count number
*\*\return pointer of the memery
**/
void Memset(void* s, uint8_t c, uint32_t count)
{
    char* xs = (char*)s;

    while (count--) /* clear 17byte buffer */
    {
        *xs++ = c;
    }

    return;
}

/**
*\*\name    IIC_RestoreSlaveByClock.
*\*\fun     Emulate 9 clock recovery slave by GPIO.
*\*\param   none
*\*\return  none 
**/
void IIC_RestoreSlaveByClock(void)
{
    uint8_t i;
    GPIO_InitType i2cx_gpio;
    
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
    RCC_APB2_Peripheral_Reset(RCC_APB2_PERIPH_AFIO);
    GPIO_Reset(GPIOx);
    
    GPIO_Structure_Initialize(&i2cx_gpio);
    i2cx_gpio.Pin            = I2Cx_SCL_PIN;
    i2cx_gpio.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    i2cx_gpio.GPIO_Mode      = GPIO_MODE_OUT_PP;
    GPIO_Peripheral_Initialize(GPIOx, &i2cx_gpio);
    
    for (i = 0; i < 9; i++)
    {
        GPIO_Pins_Set(GPIOx, I2Cx_SCL_PIN);
        Delay_us(5);
        GPIO_PBC_Pins_Reset(GPIOx, I2Cx_SCL_PIN);
        Delay_us(5);
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
        
        IIC_RestoreSlaveByClock();
        
        log_info("***** IIC module by RCC reset! *****\r\n");
        i2c_master_init();
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

