/************************************************************************************
 * configs/stm32f767i-fppa/src/stm32_pwm_timer.c
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
#include <sys/types.h>
#include <nuttx/drivers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "stm32_pwm.h"
#include "stm32f767i-fppa.h"



#ifdef CONFIG_PWM

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/



/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *  Initialize PWM and register the PWM device
 *
 ************************************************************************************/

int stm32_pwm_setup(void)
{
#if defined(CONFIG_STM32F7_TIM1_PWM) || defined(CONFIG_STM32F7_TIM2_PWM) || defined(CONFIG_STM32F7_TIM3_PWM) ||\
	defined(CONFIG_STM32F7_TIM4_PWM) || defined(CONFIG_STM32F7_TIM5_PWM) || defined(CONFIG_STM32F7_TIM6_PWM) ||\
	defined(CONFIG_STM32F7_TIM7_PWM) || defined(CONFIG_STM32F7_TIM8_PWM) || defined(CONFIG_STM32F7_TIM9_PWM) ||\
	defined(CONFIG_STM32F7_TIM10_PWM) || defined(CONFIG_STM32F7_TIM11_PWM) || defined(CONFIG_STM32F7_TIM12_PWM) ||\
	defined(CONFIG_STM32F7_TIM13_PWM) || defined(CONFIG_STM32F7_TIM14_PWM)
  struct pwm_lowerhalf_s *pwm;
  int ret;

#ifdef CONFIG_STM32F7_TIM1_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(1);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM1 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm1", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM1_PWM*/

#ifdef CONFIG_STM32F7_TIM2_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(2);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM2 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm2", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM2_PWM*/

#ifdef CONFIG_STM32F7_TIM3_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(3);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM3 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm3", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM3_PWM*/

#ifdef CONFIG_STM32F7_TIM4_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(4);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM4 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm4", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM4_PWM*/

#ifdef CONFIG_STM32F7_TIM5_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(5);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM5 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm5", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM5_PWM*/

#ifdef CONFIG_STM32F7_TIM6_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(6);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM3 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm6", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM6_PWM*/

#ifdef CONFIG_STM32F7_TIM7_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(7);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM7 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm7", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM7_PWM*/

#ifdef CONFIG_STM32F7_TIM8_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(8);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM8 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm8", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM8_PWM*/

#ifdef CONFIG_STM32F7_TIM9_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(9);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM9 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm9", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM9_PWM*/

#ifdef CONFIG_STM32F7_TIM10_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(10);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM10 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm10", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM10_PWM*/

#ifdef CONFIG_STM32F7_TIM11_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(11);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM11 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm11", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM11_PWM*/

#ifdef CONFIG_STM32F7_TIM12_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(12);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM12 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm12", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM12_PWM*/

#ifdef CONFIG_STM32F7_TIM13_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(13);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM13 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm13", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM13_PWM*/

#ifdef CONFIG_STM32F7_TIM14_PWM
  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

  pwm = stm32_pwminitialize(14);
  if (pwm == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM1 interface\n");
      return -ENODEV;
  }

  /* Register the PWM driver at "/dev/pwmx" */

  ret = pwm_register("/dev/pwm14", pwm);
  if (ret < 0){
      syslog(LOG_ERR,"ERROR: pwm_register failed: %d\n", ret);
      return ret;
  }
#endif	/*CONFIG_STM32F7_TIM14_PWM*/



  return OK;
#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_PWM */

