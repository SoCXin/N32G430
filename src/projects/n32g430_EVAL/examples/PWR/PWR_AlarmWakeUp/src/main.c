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
 */

#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include "log.h"
#include "n32g430_rtc.h"
#include "n32g430_pwr.h"

RTC_DateType RTC_DateStructure;
RTC_DateType RTC_DateDefault;
RTC_TimeType RTC_TimeStructure;
RTC_TimeType RTC_TimeDefault;
RTC_InitType RTC_InitStructure;
RTC_AlarmType RTC_AlarmStructure;
RTC_AlarmType RTC_AlarmDefault;
uint32_t SynchPrediv, AsynchPrediv;

extern void PLL_TrimValueLoad(void);

/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
    /* Initialize LEDs and USART on n32g430-EVAL board */
    LEDInit();
    LedOn(LED1_PIN);
    log_init();
    log_info("RTC Config\n");
    /* RTC time and Date default Value */
    RTC_Date_And_Time_Default_Value();
    /* RTC Clock Select, 1: HSE  2:LSE 3: LSI */
    RTC_CLKSource_Config(2, 0);
	
	/* RTC calendar regulate*/
    RTC_Calendar_Config(DISABLE);
	/* RTC alram regulate*/
    RTC_Alarm_Regulate(RTC_A_ALARM);
    /* Enable RTC Alarm Interrupt */
    EXTI17_RTC_Alarm_Configuration(ENABLE);
    /* Enable PWR clock */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_PWR);
    while (1)
    {
        /* Turn off LED and prin some flag imformation */
        LedOff(LED1_PIN);
        delay(400);
        log_info("\r\n start low power! \r\n");		
        /* Request to enter STOP2 mode */
        PWR_STOP2_Mode_Enter(PWR_STOP2_ENTRY_WFI);		
        /* Exit the low power, need to reconfig the system clock
			and reinitialize the required peripherals */
        SYSCLKConfig_STOP(RCC_CFG_PLLMULFCT16);
        log_init();
        log_info("\r\n Exit low power! \r\n");
		LEDInit();
        LedOn(LED1_PIN);
        delay(400);
    }
}

/**
*\*\name    SYSCLKConfig_STOP.
*\*\fun     Reconfig the system clock.
*\*\param   RCC_PLLMULL
*\*\return  none
**/
void SYSCLKConfig_STOP(uint32_t RCC_PLLMULL)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration */
    /* Enable HSE */
    RCC->CTRL |= ((uint32_t)RCC_CTRL_HSEEN);

    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC->CTRL & RCC_CTRL_HSERDF;
        StartUpCounter++;
    } while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CTRL & RCC_CTRL_HSERDF) != RESET)
    {
        HSEStatus = (uint32_t)0x01;
    }
    else
    {
        HSEStatus = (uint32_t)0x00;
    }

	PLL_TrimValueLoad();
	
    if (HSEStatus == (uint32_t)0x01)
    {
        /* Enable Prefetch Buffer */
        FLASH->AC |= FLASH_AC_PRFTBFEN;

        /* Flash 3 wait state */		
		FLASH_Latency_Set(FLASH_LATENCY_3);

        /* HCLK = SYSCLK */
        RCC->CFG |= (uint32_t)RCC_CFG_AHBPRES_DIV1;

        /* PCLK2 = HCLK */
        RCC->CFG |= (uint32_t)RCC_CFG_APB2PRES_DIV2; // RCC_CFG_APB2PRES_DIV1

        /* PCLK1 = HCLK */
        RCC->CFG |= (uint32_t)RCC_CFG_APB1PRES_DIV4; // RCC_CFG_APB1PRES_DIV2

        /*  PLL configuration: PLLCLK = HSE * 16 = 128 MHz */
        RCC->CFG &= (uint32_t)((uint32_t) ~(RCC_CFG_PLLSRC | RCC_CFG_PLLHSEPRES | RCC_CFG_PLLMULFCT));
        RCC->CFG |= (uint32_t)(RCC_CFG_PLLSRC_HSE | RCC_PLLMULL);

        /* Enable PLL */
        RCC->CTRL |= RCC_CTRL_PLLEN;

        /* Wait till PLL is ready */
        while ((RCC->CTRL & RCC_CTRL_PLLRDF) == 0)
        {
        }

        /* Select PLL as system clock source */
        RCC->CFG &= (uint32_t)((uint32_t) ~(RCC_CFG_SCLKSW));
        RCC->CFG |= (uint32_t)RCC_CFG_SCLKSW_PLL;

        /* Wait till PLL is used as system clock source */
        while ((RCC->CFG & (uint32_t)RCC_CFG_SCLKSTS) != (uint32_t)0x08)
        {
        }
    }
    else
    { /* If HSE fails to start-up, the application will have wrong clock
           configuration. User can add here some code to deal with this error */
    }
}

/**
*\*\name    delay.
*\*\fun     Delay function.
*\*\param   nCount
*\*\return  none
**/
void delay(vu32 nCount)
{
    vu32 index = 0;
    for (index = (34000 * nCount); index != 0; index--)
    {
    }
}

/**
*\*\name    EXTI17_RTC_Alarm_Configuration.
*\*\fun     Configuration RTC alarm EXTI.
*\*\param   Cmd
*\*\return  none
**/
void EXTI17_RTC_Alarm_Configuration(FunctionalState Cmd)
{
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    EXTI_Flag_Status_Clear(EXTI_LINE17);
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE17;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Peripheral_Initializes(&EXTI_InitStructure);

    /* Enable the RTC Alarm Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = RTCAlarm_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = Cmd;
    NVIC_Initializes(&NVIC_InitStructure);
}

/**
*\*\name    LedBlink.
*\*\fun     Toggles the selected Led.
*\*\param   GPIOx
*\*\param   Pin
*\*\return  none
**/
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

/**
*\*\name    LedOn.
*\*\fun     Turns selected Led on.
*\*\param   Pin
*\*\return  none
**/
void LedOn(uint16_t Pin)
{
    GPIOA->PBSC = Pin;
}

/**
*\*\name    LedOff.
*\*\fun     Turns selected Led Off.
*\*\param   Pin
*\*\return  none
**/
void LedOff(uint16_t Pin)
{
    GPIOA->PBC = Pin;
}

/**
*\*\name    LEDInit.
*\*\fun     Configures LED GPIO.
*\*\param   none
*\*\return  none
**/
void LEDInit(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.Pin        = LED1_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUT_PP;

    GPIO_Peripheral_Initialize(LED1_PORT, &GPIO_InitStructure);
}

/**
*\*\name    RTC_Alarm_Show.
*\*\fun     Display the current time on the Hyperterminal.
*\*\param   AlarmX
*\*\return  none
**/
void RTC_Alarm_Show(uint8_t AlarmX)
{
    /* Get the current Alarm */
    if (AlarmX == 0x01)
        RTC_Alarm_Get(RTC_FORMAT_BIN, RTC_A_ALARM, &RTC_AlarmStructure);
    else
        RTC_Alarm_Get(RTC_FORMAT_BIN, RTC_B_ALARM, &RTC_AlarmStructure);
    printf("\n\r //=========== Current Alarm Display ==============// \n\r");
    printf("\n\r The current alarm is :  %0.2d:%0.2d:%0.2d \n\r",
           RTC_AlarmStructure.AlarmTime.Hours,
           RTC_AlarmStructure.AlarmTime.Minutes,
           RTC_AlarmStructure.AlarmTime.Seconds);
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
*\*\fun     RTC time date and Alarm Value config.
*\*\param   none
*\*\return  none
**/
void RTC_Date_And_Time_Default_Value(void)
{
    // Date
    RTC_DateDefault.WeekDay = 3;
    RTC_DateDefault.Date    = 20;
    RTC_DateDefault.Month   = 11;
    RTC_DateDefault.Year    = 19;
    // Time
    RTC_TimeDefault.H12     = RTC_AM_H12;
    RTC_TimeDefault.Hours   = 4;
    RTC_TimeDefault.Minutes = 22;
    RTC_TimeDefault.Seconds = 10;

    // Alarm
    RTC_AlarmDefault.AlarmTime.H12     = RTC_AM_H12;
    RTC_AlarmDefault.AlarmTime.Hours   = 4;
    RTC_AlarmDefault.AlarmTime.Minutes = 22;
    RTC_AlarmDefault.AlarmTime.Seconds = 16;
    RTC_AlarmDefault.DateWeekMode      = RTC_ALARM_SEL_WEEKDAY_DATE;
    //RTC_AlarmDefault.DateWeekValue    = 0x31;
    //RTC_AlarmDefault.AlarmMask = RTC_ALARMMASK_MINUTES;
}

/**
*\*\name    RTC_Alarm_Regulate.
*\*\fun     RTC alarm value config,and print the config information.
*\*\param   RTC_Alarm
*\*\return  SUCCESS or ERROR
**/
ErrorStatus RTC_Alarm_Regulate(uint32_t RTC_Alarm)
{
    uint32_t tmp_hh = 0xFF, tmp_mm = 0xFF, tmp_ss = 0xFF;

    /* Disable the AlarmX */
    RTC_Alarm_Disable(RTC_Alarm);

    printf("\n\r //==============Alarm X Settings================// \n\r");
    RTC_AlarmStructure.AlarmTime.H12 = RTC_AM_H12;
    RTC_TimeStructure.H12            = RTC_AM_H12;

    printf("\n\r Please Set Alarm Hours \n\r");
    tmp_hh = RTC_AlarmDefault.AlarmTime.Hours;
    if (tmp_hh == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_AlarmStructure.AlarmTime.Hours = tmp_hh;
    }
    printf(": %0.2d\n\r", tmp_hh);

    printf("\n\r Please Set Alarm Minutes \n\r");
    tmp_mm = RTC_AlarmDefault.AlarmTime.Minutes;
    if (tmp_mm == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_AlarmStructure.AlarmTime.Minutes = tmp_mm;
    }
    printf(": %0.2d\n\r", tmp_mm);

    printf("\n\r Please Set Alarm Seconds \n\r");
    tmp_ss = RTC_AlarmDefault.AlarmTime.Seconds;
    if (tmp_ss == 0xff)
    {
        return ERROR;
    }
    else
    {
        RTC_AlarmStructure.AlarmTime.Seconds = tmp_ss;
    }
    printf(": %0.2d\n\r", tmp_ss);

    /* Set the Alarm X */
    RTC_AlarmStructure.DateWeekValue = 0x31;

    RTC_AlarmStructure.DateWeekMode = RTC_AlarmDefault.DateWeekMode;

    // RTC_AlarmStructure.AlarmMask = RTC_ALARMMASK_WEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_SECONDS;
    RTC_AlarmStructure.AlarmMask = RTC_ALARMMASK_WEEKDAY | RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
    // RTC_AlarmStructure.AlarmMask = RTC_ALARMMASK_WEEKDAY;

    /* Configure the RTC Alarm A register */
    RTC_Alarm_Set(RTC_FORMAT_BIN, RTC_Alarm, &RTC_AlarmStructure);
    printf("\n\r>> !! RTC Set Alarm_X success. !! <<\n\r");

    if (RTC_Alarm == RTC_A_ALARM)
    {
        /* Enable the RTC Alarm A Interrupt */
        RTC_Interrupts_Enable(RTC_INT_ALRA);
        RTC_Alarm_Show(1);
    }
    else
    {
        /* Enable the RTC Alarm B Interrupt */
        RTC_Interrupts_Enable(RTC_INT_ALRB);
        RTC_Alarm_Show(2);
    }
    /* Enable the alarm */
    RTC_Alarm_Enable(RTC_Alarm);
    return SUCCESS;
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
    if (RTC_Calendar_Initializes(RTC_FORMAT_BIN, &RTC_InitStructure, &RTC_DateStructure, &RTC_TimeStructure, delay_cmd) == ERROR)
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
*\*\                - if RTC_PM_H12 is select the value in the 0-12 range
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
*\*\            - 1     clock source select HSE/128
*\*\            - 2     clock source select LSE
*\*\            - 3     clock source select LSI
*\*\param   FirstLastCfg
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

    if (ClkSrc == 0x01)
    {
        log_info("\r\n RTC_ClkSrc Is Set HSE128! \r\n");
        if (FirstLastCfg == 0)
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
    else if (ClkSrc == 0x02)
    {
        log_info("\r\n RTC_ClkSrc Is Set LSE! \r\n");

        if (FirstLastCfg == 0)
        {
            /* Enable the LSE OSC32_IN PC14 */
            RCC_LSI_Disable(); // LSI is turned off here to ensure that only one clock is turned on

#if (_TEST_LSE_BYPASS_)
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

#if (_TEST_LSE_BYPASS_)
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
    else if (ClkSrc == 0x03)
    {
        log_info("\r\n RTC_ClkSrc Is Set LSI! \r\n");
        if (FirstLastCfg == 0)
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



