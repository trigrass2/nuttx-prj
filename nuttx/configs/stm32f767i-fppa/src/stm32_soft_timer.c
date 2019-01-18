/************************************************************************************
 * configs/stm32f4discovery/src/stm32_soft_timer.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/timers/timer.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32_tim.h"
#include "stm32f767i-fppa.h"

#ifdef CONFIG_TIMER

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/




/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_timer_setup
 *
 * Description:
 *  Initialize soft timer and register the timer device
 *
 ************************************************************************************/

int stm32_timer_setup(void)
{
#if defined(CONFIG_STM32F7_TIM1) || defined(CONFIG_STM32F7_TIM2) || defined(CONFIG_STM32F7_TIM3) ||\
    defined(CONFIG_STM32F7_TIM4) || defined(CONFIG_STM32F7_TIM5) || defined(CONFIG_STM32F7_TIM6) ||\
    defined(CONFIG_STM32F7_TIM7) || defined(CONFIG_STM32F7_TIM8) || defined(CONFIG_STM32F7_TIM9) ||\
    defined(CONFIG_STM32F7_TIM10) || defined(CONFIG_STM32F7_TIM11) || defined(CONFIG_STM32F7_TIM12) ||\
    defined(CONFIG_STM32F7_TIM13) || defined(CONFIG_STM32F7_TIM14)
  int ret;

  /* Call stm32_timer_initialize() to get an instance of the CAN interface and Register the CAN driver at "/dev/timerx" */

#if defined(CONFIG_TIMER_PORT_6)
  ret = stm32_timer_initialize(CONFIG_TIMER_PATH_6, 6);

  if (ret < 0)
    {
      syslog(LOG_ERR,"[TIMER]: Failed to initialize timer6  failed: %d\n", ret);
      return ret;
    }
#endif /*defined(CONFIG_TIMER_PORT_6)*/

#if defined(CONFIG_TIMER_PORT_7)
  ret = stm32_timer_initialize(CONFIG_TIMER_PATH_7, 7);

  if (ret < 0)
    {
      syslog(LOG_ERR,"[TIMER]: Failed to initialize timer7 : %d\n", ret);
      return ret;
    }
#endif /*defined(CONFIG_TIMER_PORT_7)*/
  ret = OK;
  return ret;
#else
  ret = -ENODEV;
  return ret;
#endif	/*	#if defined(CONFIG_STM32F7_TIM1) || defined(CONFIG_STM32F7_TIM2) || defined(CONFIG_STM32F7_TIM3)\
    defined(CONFIG_STM32F7_TIM4) || defined(CONFIG_STM32F7_TIM5) || defined(CONFIG_STM32F7_TIM6)\
    defined(CONFIG_STM32F7_TIM7) || defined(CONFIG_STM32F7_TIM8) || defined(CONFIG_STM32F7_TIM9)\
    defined(CONFIG_STM32F7_TIM10) || defined(CONFIG_STM32F7_TIM11) || defined(CONFIG_STM32F7_TIM12)\
    defined(CONFIG_STM32F7_TIM13) || defined(CONFIG_STM32F7_TIM14)	*/
}

#endif /* CONFIG_TIMER */

