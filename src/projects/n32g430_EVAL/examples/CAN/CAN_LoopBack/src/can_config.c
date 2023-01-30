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
*\*\file can_config.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/ 

#include "can_config.h"

#define CAN_TXDLC_8    ((uint8_t)8)
#define CAN_FILTERNUM0 ((uint8_t)0)

CanTxMessage CAN_TxMessage;
CanRxMessage CAN_RxMessage;


CAN_InitType        CAN_InitStructure;
CAN_FilterInitType  CAN_FilterInitStructure;


/**
*\*\name   CAN_CONFIG
*\*\fun    CAN configure.
*\*\param  none
*\*\return none
**/
void CAN_CONFIG(void)
{
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_CAN);
    
    /* Initializes CAN*/
    CAN_NVIC_Configuration();
    CAN_GPIO_Configuration();
    
    CAN_Reset(CAN);
    CAN_Structure_Initializes(&CAN_InitStructure);
    /* CAN cell init */
    CAN_InitStructure.TTCM          = DISABLE;
    CAN_InitStructure.ABOM          = DISABLE;
    CAN_InitStructure.AWKUM         = DISABLE;
    CAN_InitStructure.NART          = DISABLE;
    CAN_InitStructure.RFLM          = DISABLE;
    CAN_InitStructure.TXFP          = ENABLE;
    CAN_InitStructure.OperatingMode = CAN_LOOPBACK_MODE;
    CAN_InitStructure.RSJW          = CAN_RSJW_1TQ;
    CAN_InitStructure.TBS1          = CAN_TBS1_8TQ;
    CAN_InitStructure.TBS2          = CAN_TBS2_7TQ;   
    CAN_InitStructure.BaudRatePrescaler = 4;
    
    /* Initializes the CAN */
    CAN_Initializes(CAN, &CAN_InitStructure); 
    
    
    /* CAN filter init */
    CAN_FilterInitStructure.Filter_Num            = 0;
    CAN_FilterInitStructure.Filter_Mode           = CAN_FILTER_IDLISTMODE;
    CAN_FilterInitStructure.Filter_Scale          = CAN_FILTER_32BITSCALE;
    CAN_FilterInitStructure.Filter_HighId         = 0x8000;
    CAN_FilterInitStructure.Filter_LowId          = 0x0000;
    CAN_FilterInitStructure.FilterMask_HighId     = 0x4000;
    CAN_FilterInitStructure.FilterMask_LowId      = 0x0000;
    CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
    CAN_FilterInitStructure.Filter_Act            = ENABLE;
    CAN_Filter_Initializes(&CAN_FilterInitStructure);
    /* IT Configuration for CAN */  
    CAN_Config_Interrupt_Enable(CAN, CAN_INT_FMP0);
}


/**
*\*\name   CAN_NVIC_Configuration
*\*\fun    CAN NVIC configure.
*\*\param  none
*\*\return none
**/
void CAN_NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;
    
    /* Initializes the NVIC */
    NVIC_Initializes(&NVIC_InitStructure);
    
    /* NVIC configure */
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}


/**
*\*\name   CAN_GPIO_Configuration
*\*\fun    CAN GPIO configure.
*\*\param  none
*\*\return none
**/
void CAN_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initializes the GPIO */
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    
    /* configure CAN pin */
	RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
	RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);

    GPIO_InitStructure.Pin            = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_CAN;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin             = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF6_CAN;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
}


/**
*\*\name    Check_CanRecData
*\*\fun     Check Can Receive Date
*\*\param   receive_message :
*\*\            - StdId
*\*\                - It ranges from 0 to 0x7FF
*\*\            - ExtId
*\*\                - It ranges from 0 to 0x1FFFFFFF
*\*\            - IDE
*\*\                - CAN_STANDARD_ID
*\*\                - CAN_EXTENDED_ID
*\*\            - RTR
*\*\                - CAN_RTRQ_DATA
*\*\                - CAN_RTRQ_REMOTE
*\*\            - DLC
*\*\                - It ranges from 0 to 8
*\*\            - Data[0-7]
*\*\                - It ranges from 0 to 0xFF
*\*\            - FMI
*\*\                - It ranges from 0 to 0xFF             
*\*\return  Pass / Fail
**/
uint8_t Check_CanRecData(CanRxMessage* RxMessage, uint32_t StdId, uint32_t ExtId, uint8_t IDE, uint8_t RTR, uint8_t DLC,
                         uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3,
                         uint8_t Data4, uint8_t Data5, uint8_t Data6, uint8_t Data7, uint8_t FMI)
{
    /* Receive */

    /* ID */
    if(IDE == CAN_EXTENDED_ID)
    {
        if(RxMessage->ExtId   != ExtId)          
        {
            return Fail;
        }
    }
    else if(IDE == CAN_STANDARD_ID)
    {
        if(RxMessage->StdId   != StdId)  
        {
            return Fail;
        }
    }
    /* **** */

    /* IDE/RTR/DLC */
    if( (RxMessage->IDE     != IDE)   ||         /* CAN_ID_STD / CAN_ID_EXT */
        (RxMessage->RTR     != RTR)   ||         /* CAN_RTR_DATA / CAN_RTR_REMOTE */
        (RxMessage->DLC     != DLC)              /* 0 to 8 */
    )
    {
        return Fail;
    }
    /* **** */

    /* DATA */
    if(RTR == CAN_RTRQ_DATA)
    {
        if(DLC >= 1)
        {
            if(RxMessage->Data[0] != Data0)
            {
                return Fail;
            }
        }
        if(DLC >= 2)
        {
            if(RxMessage->Data[1] != Data1)
            {
                return Fail;
            }
        }
        if(DLC >= 3)
        {
            if(RxMessage->Data[2] != Data2)
            {
                return Fail;
            }
        }
        if(DLC >= 4)
        {
            if(RxMessage->Data[3] != Data3)
            {
                return Fail;
            }
        }
        if(DLC >= 5)
        {
            if(RxMessage->Data[4] != Data4)
            {
                return Fail;
            }
        }
        if(DLC >= 6)
        {
            if(RxMessage->Data[5] != Data5)
            {
                return Fail;
            }
        }
        if(DLC >= 7)
        {
            if(RxMessage->Data[6] != Data6)
            {
                return Fail;
            }
        }
        if(DLC == 8)
        {
            if(RxMessage->Data[7] != Data7)
            {
                return Fail;
            }
        }
        if(DLC > 8)
        {
            return Fail;
        }
    }
    else if(RTR == CAN_RTRQ_REMOTE)
    {
        
    }

    /* FMI */
    if(RxMessage->FMI != FMI)           
    {
        return Fail;
    }

    return Pass;
}


