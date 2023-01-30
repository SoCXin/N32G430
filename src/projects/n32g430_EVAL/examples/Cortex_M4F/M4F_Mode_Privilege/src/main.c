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

#include "main.h"
#include "log.h"
#include <stdio.h>

/**
**  Cortex-M4F Mode Privilege
**/

#define SP_PROCESS_SIZE          0x200 /* Process stack size */
#define SP_PROCESS               0x02  /* Process stack */
#define SP_MAIN                  0x00  /* Main stack */
#define THREAD_MODE_PRIVILEGED   0x00  /* Thread mode has privileged access */
#define THREAD_MODE_UNPRIVILEGED 0x01  /* Thread mode has unprivileged access */

/* clang-format off */
#if defined ( __CC_ARM   )
  __ASM void __SVC(void) 
  { 
    SVC 0x01 
    BX R14
  }
#elif defined ( __ICCARM__ )
  static __INLINE  void __SVC()                     { __ASM ("svc 0x01");}
#elif defined   (  __GNUC__  )
  static __INLINE void __SVC()                      { __ASM volatile ("svc 0x01");}
#endif
/* clang-format on */

__IO uint8_t PSPMemAlloc[SP_PROCESS_SIZE];
__IO uint32_t Index = 0, PSPValue = 0, CurrentStack = 0, ThreadMode = 0;

  
/**
*\*\name   main
*\*\fun    Main program.
*\*\return none
**/
int main(void)
{    
    /* USART Init */
    log_init();
    
    printf("Cortex-M4F Mode Privilege \r\n");
    
    /* Switch Thread mode Stack from Main to Process */
    /* Initialize memory reserved for Process Stack */
    for (Index = 0; Index < SP_PROCESS_SIZE; Index++)
    {
        PSPMemAlloc[Index] = 0x00;
    }

    /* Set Process stack value */
    __set_PSP((uint32_t)PSPMemAlloc + SP_PROCESS_SIZE);

    /* Select Process Stack as Thread mode Stack */
    __set_CONTROL(SP_PROCESS);

    /* Get the Thread mode stack used */
    if ((__get_CONTROL() & 0x02) == SP_MAIN)
    {
        /* Main stack is used as the current stack */
        CurrentStack = SP_MAIN;
    }
    else
    {
        /* Process stack is used as the current stack */
        CurrentStack = SP_PROCESS;

        /* Get process stack pointer value */
        PSPValue = __get_PSP();
    }

    /* Switch Thread mode from privileged to unprivileged */
    /* Thread mode has unprivileged access */
    __set_CONTROL(THREAD_MODE_UNPRIVILEGED | SP_PROCESS);

    /* Unprivileged access mainly affect ability to:
        - Use or not use certain instructions such as MSR fields
        - Access System Control Space (SCS) registers such as NVIC and SysTick */

    /* Check Thread mode privilege status */
    if ((__get_CONTROL() & 0x01) == THREAD_MODE_PRIVILEGED)
    {
        /* Thread mode has privileged access  */
        ThreadMode = THREAD_MODE_PRIVILEGED;
    }
    else
    {
        /* Thread mode has unprivileged access*/
        ThreadMode = THREAD_MODE_UNPRIVILEGED;
    }

    /* Switch back Thread mode from unprivileged to privileged */
    /* Try to switch back Thread mode to privileged (Not possible, this can be
       done only in Handler mode) */
    __set_CONTROL(THREAD_MODE_PRIVILEGED | SP_PROCESS);

    /* Generate a system call exception, and in the INTSTS switch back Thread mode to privileged */
    __SVC();

    /* Check Thread mode privilege status */
    if ((__get_CONTROL() & 0x01) == THREAD_MODE_PRIVILEGED)
    {
        /* Thread mode has privileged access */
        ThreadMode = THREAD_MODE_PRIVILEGED;
    }
    else
    {
        /* Thread mode has unprivileged access */
        ThreadMode = THREAD_MODE_UNPRIVILEGED;
    }

    while (1)
    {
    }
  
}

