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
*\*\file can_config.h
*\*\author Nations
*\*\version v1.0.1
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/ 
 
#ifndef __CAN_CONFIG_H__
#define __CAN_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g430.h"
#include "n32g430_conf.h"

typedef enum
{
    Fail   = (uint8_t)0,
    Pass   = (uint8_t)1
} Slave_Return;

void CAN_NVIC_Configuration(void);
void CAN_GPIO_Configuration(void);
void CAN_CONFIG(void);

uint8_t Check_CanRecData(CanRxMessage* RxMessage, uint32_t StdId, uint32_t ExtId, uint8_t IDE, uint8_t RTR, uint8_t DLC,
                         uint8_t Data0, uint8_t Data1, uint8_t Data2, uint8_t Data3,
                         uint8_t Data4, uint8_t Data5, uint8_t Data6, uint8_t Data7, uint8_t FMI);

extern CanTxMessage CAN_TxMessage;
extern CanRxMessage CAN_RxMessage;

extern CAN_InitType        CAN_InitStructure;
extern CAN_FilterInitType  CAN_FilterInitStructure;

#ifdef __cplusplus
}
#endif

#endif /* __CAN_CONFIG_H__ */


