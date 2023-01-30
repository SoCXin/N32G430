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
*\*\file lin_master.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#ifndef _LIN_SLAVE_H
#define _LIN_SLAVE_H
#include <stdio.h>
#include "n32g430.h"

typedef void (*FUN_RESP)(uint8_t *pData, uint8_t Len); 

/* Exported Functions --------------------------------------------------------*/
#define GET_PID(data) ((data&0x3F)|((((data&0x01)^((data>>1)&0x01)^((data>>2)&0x01)^((data>>4)&0x01))&0x01)<<6)|(((~(((data>>1)&0x01)^((data>>3)&0x01)^((data>>4)&0x01)^((data>>5)&0x01)))&0x01)<<7))
#define ID_TYPE_SR  0
#define ID_TYPE_SW  1
/* Define the frame ID for sending and receiving data. The frame ID must be consistent with the software configuration of the bit-computer; 
otherwise, the frame ID cannot work properly.
For the LIN bus, the data sending and receiving ID can be defined as one ID or different ID */
#define MSG_RECEIVE_ID  0x3C
#define MSG_SEND_ID     0x3D
// Define the data check type, 0- standard check, 1- enhanced check, only for LIN upgrade
#define CHECK_TYPE   0
//Firmware type value definition
#define FW_TYPE_BOOT     0x55
#define FW_TYPE_APP      0xAA
//Set the current firmware type to BOOT
#define FW_TYPE         FW_TYPE_APP

typedef enum
{
    IDLE,
    SYNCH,
    ID_LEN,
    DATA_GET,
    CHECKSUM
} LIN_STATE;

typedef struct _LIN_EX_MSG
{
    unsigned char DataLen;      //Number of valid data bytes in a data segment
    unsigned char Sync;         
    unsigned char PID;          
    unsigned char Data[8];      
    unsigned char Check;        
} LIN_EX_MSG;
void LIN_Configuration(int BaudRate);
void LIN_SetResp(uint8_t ID, uint8_t *pData, uint8_t Len, uint8_t CheckType);
void LIN_IRQHandler(void);
void BOOT_ExecutiveCommand(uint8_t *pData, FUN_RESP pFunResp);

#endif /*LIN_DRIVER_H*/

/*********************************END OF FILE**********************************/


