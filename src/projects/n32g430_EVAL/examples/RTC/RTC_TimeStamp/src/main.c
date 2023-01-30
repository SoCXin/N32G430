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
#include <stdio.h>
#include <stdint.h>
#include "log.h"
#include "n32g430_rtc.h"
#include "n32g430_exti.h"

#define RTC_CLK_HSE128          1
#define RTC_CLK_LSE             2
#define RTC_CLK_LSI             3

#define RTC_CLK_FIRST_CONFIG    0
#define RTC_CLK_LAST_CONFIG     1


RTC_DateType RTC_DateStructure;
RTC_DateType RTC_DateDefault;
RTC_TimeType RTC_TimeStructure;
RTC_TimeType RTC_TimeDefault;
RTC_InitType RTC_InitStructure;
RTC_AlarmType RTC_AlarmStructure;
RTC_AlarmType RTC_AlarmDefault;
uint32_t SynchPrediv, AsynchPrediv;

void RTC_CLKSource_Config(uint8_t ClkSrc, uint8_t FirstLastCfg);
ErrorStatus RTC_Date_Regulate(RTC_DateType* RTC_DateStruct);
ErrorStatus RTC_Time_Regulate(RTC_TimeType* RTC_TimeStruct);
ErrorStatus RTC_Calendar_Config(FunctionalState delay_cmd);
ErrorStatus RTC_Alarm_Regulate(uint32_t RTC_Alarm);
void RTC_Date_And_Time_Default_Value(void);
static void RTC_Prescaler_Config(RTC_InitType *RTC_InitStructure);

void EXTI18_TimeStamp_IRQn_Configuration(FunctionalState NewState,uint32_t TSEdgeSel);
void EXTI_PB8_TimeStamp_Configuration(void);

/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
    /* Initialize USART,TX: PA9 */
    log_init();
    log_info("RTC Init");
    
    /* RTC date time default value */
    RTC_Date_And_Time_Default_Value();
    
    /* RTC clock source select */
    RTC_CLKSource_Config(RTC_CLK_LSE, RTC_CLK_FIRST_CONFIG);

    /* RTC calendar regulate */
    RTC_Calendar_Config(DISABLE);
    
    /* Calibrate output 1Hz signal */
    RTC_Calibration_Output_Config(RTC_CALIB_OUTPUT_1HZ);
    /* Calibrate output config,push pull */
    RTC_Output_Mode_Config(RTC_OUTPUT_PUSHPULL);
    /* Calibrate output enable */
    RTC_Calibration_Output_Enable();

    /* Configure EXTI PB8 pin  connected to RTC TimeStamp
    (while externally feeding PB8 with 1HZ signal output from PC13)
    */
    EXTI_PB8_TimeStamp_Configuration();
    EXTI18_TimeStamp_IRQn_Configuration(ENABLE, 1);
    /* clear RTC time stamp flag  */
    RTC_Flag_Clear(RTC_FLAG_TISF);
    RTC_Flag_Clear(RTC_FLAG_TISOVF);
 
    RTC_TimeStamp_Enable(RTC_TIMESTAMP_EDGE_RISING);
    RTC_Interrupts_Enable(RTC_INT_TST);
    while (1)
    {
    }
}

/**
*\*\name    EXTI_PB8_TimeStamp_Configuration
*\*\fun     EXTI PB8 I/O config and use the EXTI interrupt to trigger time stamp.
*\*\param   none
*\*\return  none
**/
void EXTI_PB8_TimeStamp_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);

    /* Enable the AFIO Clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);

    /* Configure PB8 in alternate function mode */
    GPIO_InitStructure.Pin        = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

    /* Connect PB8 to RTC_AF1 */
    EXTI_RTC_Time_Stamp_Select(EXTI_TSSEL_LINE8);
    
    /* Connect EXTI8 Line to PB8 pin */
    GPIO_EXTI_Line_Set(EXTI_LINE_SOURCE8, AFIO_EXTI_PB8);
    
}

/**
*\*\name    EXTI18_TimeStamp_IRQn_Configuration
*\*\fun     Configure the timestamp interruput,EXTI18.
*\*\param   NewState
                - ENABLE
                - DISABLE
*\*\param   TSEdgeSel
                - 1 : EXTI_Trigger_Rising
                - 2 : EXTI_Trigger_Falling
*\*\return  none
**/
void EXTI18_TimeStamp_IRQn_Configuration(FunctionalState NewState,uint32_t TSEdgeSel)
{
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    
    /* Configure Button EXTI line */
    EXTI_Interrupt_Status_Clear(EXTI_LINE18);
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE18;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    if(TSEdgeSel == 0x01)
        EXTI_InitStructure.EXTI_Trigger   = EXTI_Trigger_Rising;  
    else if(TSEdgeSel == 0x02)
        EXTI_InitStructure.EXTI_Trigger   = EXTI_Trigger_Falling;
    else
        log_info("\r\n The TSEdgeSel value is error! \r\n");
    EXTI_InitStructure.EXTI_LineCmd                       = ENABLE;
    EXTI_Peripheral_Initializes(&EXTI_InitStructure);
    
    /* Enable the RTC tamper stamp Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                    = RTC_TAMPER_STAMP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                 = NewState;
    NVIC_Initializes(&NVIC_InitStructure);
}

/**
*\*\name    RTC_TimeStampShow
*\*\fun     Display the current TimeStamp (time and date) on the Hyperterminal.
*\*\param   none
*\*\return  none
**/
void RTC_TimeStamp_Show(void)
{
    RTC_DateType RTC_TimeStampDateStructure;
    RTC_TimeType RTC_TimeStampStructure;
    /* Get the current TimeStamp */
    RTC_TimeStamp_Get(RTC_FORMAT_BIN, &RTC_TimeStampStructure, &RTC_TimeStampDateStructure);
    log_info("\n\r //=========TimeStamp Display (Time and Date)============// \n\r");
    log_info("\n\r The current time stamp time (Hour-Minute-Second) is :  %0.2d:%0.2d:%0.2d \n\r",
           RTC_TimeStampStructure.Hours,
           RTC_TimeStampStructure.Minutes,
           RTC_TimeStampStructure.Seconds);
    log_info("\n\r The current timestamp date (WeekDay-Date-Month) is :  %0.2d-%0.2d-%0.2d \n\r",
           RTC_TimeStampDateStructure.WeekDay,
           RTC_TimeStampDateStructure.Date,
           RTC_TimeStampDateStructure.Month);
}

/**
*\*\name    RTC_Date_Show.
*\*\fun     Display the current Date on the Hyperterminal.
*\*\param   none
*\*\return  none
**/
void RTC_Date_Show(void)
{
    /* Get the current Date */
    RTC_Date_Get(RTC_FORMAT_BIN, &RTC_DateStructure);
    log_info("\n\r //=========== Current Date Display ==============// \n\r");
    log_info("\n\r The current date (WeekDay-Date-Month-Year) is :  %0.2d-%0.2d-%0.2d-%0.2d \n\r",
             RTC_DateStructure.WeekDay,
             RTC_DateStructure.Date,
             RTC_DateStructure.Month,
             RTC_DateStructure.Year);
}

/**
*\*\name    RTC_Time_Show.
*\*\fun     Display the current time on the Hyperterminal.
*\*\param   none
*\*\return  none
**/
void RTC_Time_Show(void)
{
    /* Get the current Time and Date */
    RTC_Time_Get(RTC_FORMAT_BIN, &RTC_TimeStructure);
    log_info("\n\r //============ Current Time Display ===============// \n\r");
    log_info("\n\r The current time (Hour-Minute-Second) is :  %0.2d:%0.2d:%0.2d \n\r",
             RTC_TimeStructure.Hours,
             RTC_TimeStructure.Minutes,
             RTC_TimeStructure.Seconds);
    /* Unfreeze the RTC DAT Register */
    (void)RTC->DATE;
}

/**
*\*\name    RTC_Date_And_Time_Default_Value.
*\*\fun     RTC initalize default value.
*\*\param   none
*\*\return  none
**/
void RTC_Date_And_Time_Default_Value(void)
{ // Date
    RTC_DateDefault.WeekDay = 3;
    RTC_DateDefault.Date    = 24;
    RTC_DateDefault.Month   = 11;
    RTC_DateDefault.Year    = 21;
    // Time
    RTC_TimeDefault.H12     = RTC_AM_H12;
    RTC_TimeDefault.Hours   = 4;
    RTC_TimeDefault.Minutes = 24;
    RTC_TimeDefault.Seconds = 55;
}


/**
*\*\name    RTC_Calendar_Config.
*\*\fun     RTC date regulate with the default value.
*\*\param   delay_cmd
*\*\            - ENABLE
*\*\            - DISABLE
*\*\return  ERROR or SUCCESS
**/
ErrorStatus RTC_Calendar_Config(FunctionalState delay_cmd)
{
    /* RTC prescaler regulate */
    RTC_Prescaler_Config(&RTC_InitStructure);
    /* RTC date time and alarm regulate */
    RTC_Date_Regulate(&RTC_DateStructure);
    RTC_Time_Regulate(&RTC_TimeStructure);

    /* Initializes RTC calendar */
    if (RTC_Calendar_Initializes(RTC_FORMAT_SELECT, &RTC_InitStructure, &RTC_DateStructure, &RTC_TimeStructure, delay_cmd) == ERROR)
    {
        log_info("\n\r>> !! RTC Set Calendar failed. !! <<\n\r");
        return ERROR;
    }
    else
    {
        log_info("\n\r>> !! RTC Set Calendar success. !! <<\n\r");
        RTC_Date_Show();
        RTC_Time_Show();
        return SUCCESS;
    }
}

/**
*\*\name    RTC_Date_Regulate.
*\*\fun     RTC date regulate with the default value.
*\*\param   RTC_DateStruct : RTC_DateStruct pointer to a RTC_DateType structure that contains
*\*\                       the date configuration information for the RTC.
*\*\            - WeekDay 
*\*\                - RTC_WEEKDAY_MONDAY
*\*\                - RTC_WEEKDAY_TUESDAY
*\*\                - RTC_WEEKDAY_WEDNESDAY
*\*\                - RTC_WEEKDAY_THURSDAY
*\*\                - RTC_WEEKDAY_FRIDAY
*\*\                - RTC_WEEKDAY_SATURDAY
*\*\                - RTC_WEEKDAY_SUNDAY
*\*\            - Month 
*\*\                - RTC_MONTH_JANUARY
*\*\                - RTC_MONTH_FEBRURY
*\*\                - RTC_MONTH_MARCH
*\*\                - RTC_MONTH_APRIL
*\*\                - RTC_MONTH_MAY
*\*\                - RTC_MONTH_JUNE
*\*\                - RTC_MONTH_JULY
*\*\                - RTC_MONTH_AUGUST
*\*\                - RTC_MONTH_SEPTEMBER
*\*\                - RTC_MONTH_OCTOBER
*\*\                - RTC_MONTH_NOVEMBER
*\*\                - RTC_MONTH_DECEMBER
*\*\            - Date    the value in the 1-31 range 
*\*\            - Year    the value in the 0-99 range
*\*\return  ERROR or SUCCESS
**/
ErrorStatus RTC_Date_Regulate(RTC_DateType* RTC_DateStruct)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;
    log_info("\n\r //=============Date Settings================// \n\r");

    log_info("\n\r Please Set WeekDay (01-07) \n\r");
    tmp_hh = RTC_DateDefault.WeekDay;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStruct->WeekDay = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    tmp_hh = 0xFF;
    log_info("\n\r Please Set Date (01-31) \n\r");
    tmp_hh = RTC_DateDefault.Date;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStruct->Date = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    log_info("\n\r Please Set Month (01-12)\n\r");
    tmp_mm = RTC_DateDefault.Month;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStruct->Month = tmp_mm;
    }
    log_info(": %0.2d\n\r", tmp_mm);

    log_info("\n\r Please Set Year (00-99)\n\r");
    tmp_ss = RTC_DateDefault.Year;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_DateStruct->Year = tmp_ss;
    }
    log_info(": %0.2d\n\r", tmp_ss);
    
    return SUCCESS;
}

/**
*\*\name    RTC_Time_Regulate.
*\*\fun     RTC time regulate with the default value.
*\*\param   RTC_TimeStruct : RTC_TimeStruct pointer to a RTC_TimeType structure that contains
*\*\        the time configuration information for the RTC.When the user does not need to configure this 
*\*\        parameter, note that NULL can be passed in.
*\*\            - Hours
*\*\                - if RTC_AM_H12 is select the value in the 0-23 range
*\*\                - if RTC_PM_H12 is select the value in the 1-12 range
*\*\            - Minutes  the value set in the 0-59 range
*\*\            - Seconds  the value set in the 0-59 range
*\*\            - H12
*\*\                - RTC_AM_H12
*\*\                - RTC_PM_H12
*\*\return  ERROR or SUCCESS
**/
ErrorStatus RTC_Time_Regulate(RTC_TimeType* RTC_TimeStruct)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;
    log_info("\n\r //==============Time Settings=================// \n\r");

    RTC_TimeStructure.H12 = RTC_TimeDefault.H12;

    log_info("\n\r Please Set Hours \n\r");
    tmp_hh = RTC_TimeDefault.Hours;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStruct->Hours = tmp_hh;
    }
    log_info(": %0.2d\n\r", tmp_hh);

    log_info("\n\r Please Set Minutes \n\r");
    tmp_mm = RTC_TimeDefault.Minutes;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStruct->Minutes = tmp_mm;
    }
    log_info(": %0.2d\n\r", tmp_mm);

    log_info("\n\r Please Set Seconds \n\r");
    tmp_ss = RTC_TimeDefault.Seconds;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_TimeStruct->Seconds = tmp_ss;
    }
    log_info(": %0.2d\n\r", tmp_ss);
    
    return SUCCESS;
}


/**
*\*\name    RTC_Prescaler_Config.
*\*\fun     RTC prescaler config.
*\*\param   RTC_InitStruct: pointer to a RTC_InitType structure. 
*\*\            - RTC_HourFormat
*\*\                - RTC_24HOUR_FORMAT
*\*\                - RTC_12HOUR_FORMAT
*\*\            - RTC_AsynchPrediv  the value in the 0-0x7F range
*\*\            - RTC_SynchPrediv   the value in the 0-0x7FFF range
*\*\return  none
**/
static void RTC_Prescaler_Config(RTC_InitType *RTC_InitStruct)
{
    /* Configure the RTC data register and RTC prescaler */
    RTC_InitStruct->RTC_AsynchPrediv = AsynchPrediv;
    RTC_InitStruct->RTC_SynchPrediv  = SynchPrediv;
    RTC_InitStruct->RTC_HourFormat   = RTC_24HOUR_FORMAT;
}

/**
*\*\name    RTC_CLKSource_Config.
*\*\fun     Configure the RTC peripheral by selecting the clock source.
*\*\param   ClkSrc    
*\*\            - RTC_CLK_HSE128    clock source select HSE/128
*\*\            - RTC_CLK_LSE       clock source select LSE
*\*\            - RTC_CLK_LSI       clock source select LSI
*\*\param   FirstLastCfg
*\*\            - RTC_CLK_FIRST_CONFIG
*\*\            - RTC_CLK_LAST_CONFIG
*\*\return  none
**/
void RTC_CLKSource_Config(uint8_t ClkSrc, uint8_t FirstLastCfg)
{
    /* Enable the PWR clock */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_PWR);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    /* Allow access to RTC */
    PWR_RTC_Backup_Access_Enable();
    
    RCC_Backup_Reset();
    
    /* Disable RTC clock */
    RCC_RTC_Clock_Disable();

    if (ClkSrc == RTC_CLK_HSE128)
    {
        log_info("\r\n RTC_ClkSrc Is Set HSE128! \r\n");
        if (FirstLastCfg == RTC_CLK_FIRST_CONFIG)
        {
            /* Enable HSE */
            RCC_LSI_Disable();
            RCC_HSE_Config(RCC_HSE_ENABLE);
            while (RCC_HSE_Stable_Wait() == ERROR)
            {
            }
            RCC_RTC_Clock_Config(RCC_RTCCLK_SRC_HSE_DIV128);
        }
        else
        {
            RCC_LSI_Disable();
            RCC_RTC_Clock_Config(RCC_RTCCLK_SRC_HSE_DIV128);

            /* Enable HSE */
            RCC_HSE_Config(RCC_HSE_ENABLE);

            while (RCC_HSE_Stable_Wait() == ERROR)
            {
            }
        }

        SynchPrediv  = 0x1E8; // 8M/128 = 62.5KHz
        AsynchPrediv = 0x7F;  // value range: 0-7F
    }
    else if (ClkSrc == RTC_CLK_LSE)
    {
        log_info("\r\n RTC_ClkSrc Is Set LSE! \r\n");

        if (FirstLastCfg == RTC_CLK_FIRST_CONFIG)
        {
            /* Enable the LSE OSC32_IN PC14 */
            RCC_LSI_Disable(); // LSI is turned off here to ensure that only one clock is turned on

#if (_LSE_BYPASS_)
            RCC_LSE_Config(RCC_LSE_BYPASS, 0x141);
#else
            RCC_LSE_Config(RCC_LSE_ENABLE, 0x141);
#endif

            while (RCC_Flag_Status_Get(RCC_FLAG_LSERD) == RESET)
            {
            }

            RCC_RTC_Clock_Config(RCC_RTCCLK_SRC_LSE);
        }
        else
        {
            /* Enable the LSE OSC32_IN PC14 */
            RCC_LSI_Disable();
            RCC_RTC_Clock_Config(RCC_RTCCLK_SRC_LSE);

#if (_LSE_BYPASS_)
            RCC_LSE_Config(RCC_LSE_BYPASS, 0x141);
#else
            RCC_LSE_Config(RCC_LSE_ENABLE, 0x141);
#endif

            while (RCC_Flag_Status_Get(RCC_FLAG_LSERD) == RESET)
            {
            }
        }

        SynchPrediv  = 0xFF; // 32.768KHz
        AsynchPrediv = 0x7F; // value range: 0-7F
    }
    else if (ClkSrc == RTC_CLK_LSI)
    {
        log_info("\r\n RTC_ClkSrc Is Set LSI! \r\n");
        if (FirstLastCfg == RTC_CLK_FIRST_CONFIG)
        {
            /* Enable the LSI OSC */
            RCC_LSI_Enable();

            while (RCC_Flag_Status_Get(RCC_FLAG_LSIRD) == RESET)
            {
            }

            RCC_RTC_Clock_Config(RCC_RTCCLK_SRC_LSI);
        }
        else
        {
            RCC_RTC_Clock_Config(RCC_RTCCLK_SRC_LSI);

            /* Enable the LSI OSC */
            RCC_LSI_Enable();

            while (RCC_Flag_Status_Get(RCC_FLAG_LSIRD) == RESET)
            {
            }
        }

        SynchPrediv  = 0x13B; // 39.64928KHz
        AsynchPrediv = 0x7F;  // value range: 0-7F
    }
    else
    {
        log_info("\r\n RTC_ClkSrc Value is error! \r\n");
    }

    /* Enable the RTC Clock */
    RCC_RTC_Clock_Enable();
    RTC_Wait_For_Synchronization();
}
