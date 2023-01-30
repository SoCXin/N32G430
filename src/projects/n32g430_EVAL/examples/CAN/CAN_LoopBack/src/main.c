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

uint16_t Rx_Flag = DISABLE;

/**
*\*\name   main
*\*\fun    Main program.
*\*\param  none
*\*\return none
**/
int main(void)
{
    uint32_t wait_slak;
    
    /* NVIC Set */
    NVIC_Priority_Group_Set(NVIC_PER2_SUB2_PRIORITYGROUP);
    
    /* CAN configure */
    CAN_CONFIG();
    
    /* Transmit assign */
    CAN_TxMessage.StdId   = 0x0400;        
    CAN_TxMessage.ExtId   = 0x00;        
    CAN_TxMessage.IDE     = CAN_STANDARD_ID;           /* CAN_ID_STD / CAN_ID_EXT */
    CAN_TxMessage.RTR     = CAN_RTRQ_DATA;           /* CAN_RTR_DATA / CAN_RTR_REMOTE */
    CAN_TxMessage.DLC     = 8;           /* 0 to 8 */
    CAN_TxMessage.Data[0] = 0x00;
    CAN_TxMessage.Data[1] = 0x01;
    CAN_TxMessage.Data[2] = 0x02;
    CAN_TxMessage.Data[3] = 0x03;
    CAN_TxMessage.Data[4] = 0x04;
    CAN_TxMessage.Data[5] = 0x05;
    CAN_TxMessage.Data[6] = 0x06;
    CAN_TxMessage.Data[7] = 0x07;
    while(1)
    {
        /* Transmit */
        CAN_Transmit_Message_initializes(CAN,&CAN_TxMessage);
        while(Rx_Flag == DISABLE)
        {
            Rx_Flag = Check_CanRecData(&CAN_RxMessage, 0x0400, 0x00, CAN_STANDARD_ID, CAN_RTRQ_DATA, 8,
                         0x00,0x01, 0x02, 0x03, 
                         0x04,0x05, 0x06, 0x07, 0);
        }
        
        /* Delay */
        wait_slak = 0xFFFF;
        while(wait_slak>0)
        {
            wait_slak--;
        }
        Rx_Flag = DISABLE;
    }
}



