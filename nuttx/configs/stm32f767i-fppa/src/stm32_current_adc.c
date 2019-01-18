/************************************************************************************
 * configs/stm32f4discovery/src/stm32_current_adc.c
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

#include <arch/board/board.h>
#include <nuttx/analog/adc.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32_adc.h"
#include "stm32_gpio.h"
#include "stm32f767i-fppa.h"

#ifdef CONFIG_ADC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

#ifdef CONFIG_STM32F7_ADC1
#  define ADC_PORT_1 		1
#  define ADC_PORTPATH_1 	"/dev/adc_manager1"
#  define ADC_CHAN_NUMS_1 	6
uint8_t  adc1_chanlist[ADC_CHAN_NUMS_1]={0,4,6,5,4,9};
#endif

#ifdef CONFIG_STM32F7_ADC2
#  define ADC_PORT_2 		2
#  define ADC_PORTPATH_2 	"/dev/adc_manager2"
#  define ADC_CHAN_NUMS_2 	0
uint8_t  adc2_chanlist[ADC_CHAN_NUMS_2];
#endif

#ifdef CONFIG_STM32F7_ADC3
#  define ADC_PORT_3 		3
#  define ADC_PORTPATH_3 	"/dev/adc_manager3"
#  define ADC_CHAN_NUMS_3 	1
uint8_t  adc3_chanlist[ADC_CHAN_NUMS_3]={4};
#endif



/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_adc_manager_setup
 *
 * Description:
 *  Initialize ADC and register the ADC device
 *
 ************************************************************************************/

int stm32_adc_manager_setup(void)
{

  struct adc_dev_s *adc;
  int ret;

#if defined(CONFIG_STM32F7_ADC1)
  /* Call stm32_adcinitialize() to get an instance of the ADC interface */

  stm32_configgpio(GPIO_CURRENT_AIN1);
  stm32_configgpio(GPIO_CURRENT_AIN2);
  stm32_configgpio(GPIO_CURRENT_AIN3);
  stm32_configgpio(GPIO_CURRENT_AIN4);
  stm32_configgpio(GPIO_CURRENT_AIN5);
  stm32_configgpio(GPIO_CURRENT_BUC);

  adc = stm32_adc_initialize(ADC_PORT_1,adc1_chanlist,ADC_CHAN_NUMS_1);
  if (adc == NULL)
    {
	  syslog(LOG_ERR,"[ADC]: Failed to get adc1 interface\n");
      return -ENODEV;
    }

  /* Register the adc driver at */

  ret = adc_register(ADC_PORTPATH_1, adc);
  if (ret < 0)
    {
	  syslog(LOG_ERR,"[ADC]: Failed to register adc1: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_STM32F7_ADC2)
  /* Call stm32_adcinitialize() to get an instance of the ADC interface */

  adc = stm32_adc_initialize(ADC_PORT_2,adc2_chanlist,ADC_CHAN_NUMS_2);
  if (adc == NULL)
    {
	  syslog(LOG_ERR,"[ADC]: Failed to get adc2 interface\n");
      return -ENODEV;
    }

  /* Register the adc driver */

  ret = adc_register(ADC_PORTPATH_2, adc);
  if (ret < 0)
    {
	  syslog(LOG_ERR,"[ADC]: Failed to register adc2: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_STM32F7_ADC3)
  /* Call stm32_adcinitialize() to get an instance of the ADC interface */

  stm32_configgpio(GPIO_CURRENT_AIN6);

  adc = stm32_adc_initialize(ADC_PORT_3,adc3_chanlist,ADC_CHAN_NUMS_3);
  if (adc == NULL)
    {
	  syslog(LOG_ERR,"[ADC]: Failed to get adc3 interface\n");
      return -ENODEV;
    }

  /* Register the adc driver */

  ret = adc_register(ADC_PORTPATH_3, adc);
  if (ret < 0)
    {
	  syslog(LOG_ERR,"[ADC]: Failed to register adc3: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

#endif /* CONFIG_ADC */

