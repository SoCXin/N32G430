#include "main.h"
#include "log.h"
#include <stdio.h>

#define SYSTICK_1MS             ((uint32_t)1000)
#define SYSTICK_COUNTER_DIASBLE ((uint32_t)0xFFFFFFFE)
#define SYSTICK_COUNTER_ENABLE  ((uint32_t)0x00000001)
#define SYSTICK_COUNTER_CLEAR   ((uint32_t)0x00000000)

extern int CoreMark(void);


/**
*\*\name   main
*\*\fun    Main program.
*\*\return none
**/
int main(void)
{   
    log_init();
    /* USART Init */
    printf("CoreMark Test Start\r\n");
    /* Disables the Prefetch Buffer */
    FLASH_Prefetch_Buffer_Disable();    
    /* Enable iCache */
    FLASH_ICache_Enable();
//    FLASH_ICache_Disable();
    
    /* SysTick Init */
    SysTick_Init(SYSTICK_1MS);
    
    CoreMark();

    while (1)
    {
    }   
}


/**
*\*\name   SysTick_Init
*\*\fun    SysTick tick initialize.
*\*\return none
**/
void SysTick_Init(uint32_t NUM)
{
    /* SystemCoreClock / NUM */
    if (SysTick_Config(SystemClockFrequency / NUM))
    {
        while (1)
            ;
    }
}


/**
*\*\name   SysTick_Stop_time
*\*\fun    SysTick tick stop.
*\*\return none
**/
void SysTick_Stop_time(void)
{
    SysTick->CTRL &= SYSTICK_COUNTER_DIASBLE;
    /* Clear the SysTick Counter */
    SysTick->VAL = SYSTICK_COUNTER_CLEAR;
}
