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
#ifndef _LIN_MASTER_H
#define _LIN_MASTER_H
#include <stdio.h>
#include "n32g430.h"
#include "log.h"

#define SC_RECEIVE_TIMEOUT 0x4000  /* Direction to reader */

typedef enum
{
    CLASSIC = 0,
    ENHANCED = !CLASSIC
} ChecksumType;

typedef struct M_LIN_EX_MSG
{
    ChecksumType CheckType;     //LIN data check type
    unsigned char DataLen;      //Number of valid data bytes in a data segment
    unsigned char Sync;         
    unsigned char PID;          
    unsigned char Data[8];      
    unsigned char Check;        
} M_LIN_EX_MSG;

void SetFrameMsg(M_LIN_EX_MSG *dst_Msg, M_LIN_EX_MSG src_Msg);
void SetFramePID(M_LIN_EX_MSG *src_Msg);
uint8_t MasterGetCheckSum(uint8_t *pData, uint8_t len);
void SetFrameChecksum(M_LIN_EX_MSG *Msg);
void MasterSendBytes(uint8_t *pBuf, uint8_t Len);
void MasterSendFrame(M_LIN_EX_MSG     Msg);
void FrameHandle(void);
static ErrorStatus USART_ByteReceive(uint8_t *Data, uint32_t TimeOut);
uint32_t Master_RecData(uint8_t *pdata, uint8_t length);
ErrorStatus WaitFrameRes(uint8_t *dst_data, uint8_t length);
void TestMasterReqFrame(void);
void TestSlaveResFrame(void);
void TestLinMaster(void);

#endif
