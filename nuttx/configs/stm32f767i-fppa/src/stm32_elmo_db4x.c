/************************************************************************************
 * configs/stm32f4discovery/src/stm32_can.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <arch/board/board.h>
#include <nuttx/power/motor.h>

#include "stm32f767i-fppa.h"

#if defined(CONFIG_MOTOR_LOWER_CANBUS)
	#include "stm32_can.h"
	#include <nuttx/power/elmo_db4x.h>
#endif

#if defined(CONFIG_MOTOR_LOWER_PWM_IO)
	#include "stm32_gpio.h"
	#include "stm32_pwm.h"
	#include <nuttx/power/elmo_db4x_pwm_io.h>
#endif




/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/
#if defined(CONFIG_MOTOR_LOWER_CANBUS)
const char *str_path[]={
		"/dev/motor1",
		"/dev/motor2",
		"/dev/motor3",
		"/dev/motor4",
		"/dev/motor5",
		"/dev/motor6",
		"/dev/motor7",
		"/dev/motor8"
};
#endif /* defined(CONFIG_MOTOR_LOWER_CANBUS) */

#if defined(CONFIG_MOTOR_LOWER_PWM_IO)
const char *pwm_str_path[]={
		"/dev/motor_pwm1",
		"/dev/motor_pwm2",
		"/dev/motor_pwm3",
		"/dev/motor_pwm4",
		"/dev/motor_pwm5",
		"/dev/motor_pwm6"
};


enum tim_e
{
	TIM_8 = 0,
	TIM_5,
	TIM_MAX,
	TIM_CH1 = 1,
	TIM_CH2 = 2,
	TIM_CH3 = 3,
	TIM_CH4 = 4
};

struct pwm_info_s g_pwm_info[CONFIG_MOTOR_LOWER_PWM_IO_NUMS] =
{
#if CONFIG_MOTOR_LOWER_PWM_IO_NUMS > 0
	{
		  .frequency = 0,

	#ifdef CONFIG_PWM_MULTICHAN
		  .channels[TIM_8].channel = TIM_CH1,
		  .channels[TIM_8].duty = 0xffff>>1,
	#if CONFIG_PWM_NCHANNELS > 1
		  .channels[TIM_8].channel = TIM_CH2,
		  .channels[TIM_8].duty = 0xffff>>1,
	#endif
	#if CONFIG_PWM_NCHANNELS > 2
		  .channels[TIM_8].channel = TIM_CH3,
		  .channels[TIM_8].duty = 0xffff>>1,
	#endif
	#if CONFIG_PWM_NCHANNELS > 3
		  .channels[TIM_8].channel = TIM_CH4,
		  .channels[TIM_8].duty = 0xffff>>1,
	#endif
	#else
	  .duty = 0xffff >> 1,
	#endif /* CONFIG_PWM_MULTICHAN */

	#  ifdef CONFIG_PWM_PULSECOUNT
	 .count = 0
	#  endif

	}
#endif
#if CONFIG_MOTOR_LOWER_PWM_IO_NUMS > 1
	,{
		  .frequency = 0,

	#ifdef CONFIG_PWM_MULTICHAN
		  .channels[TIM_5].channel = TIM_CH1,
		  .channels[TIM_5].duty = 0xffff>>1,
	#if CONFIG_PWM_NCHANNELS > 1
		  .channels[TIM_5].channel = TIM_CH2,
		  .channels[TIM_5].duty = 0xffff>>1,
	#endif
	#if CONFIG_PWM_NCHANNELS > 2
		  .channels[TIM_5].channel = TIM_CH3,
		  .channels[TIM_5].duty = 0xffff>>1,
	#endif
	#if CONFIG_PWM_NCHANNELS > 3
		  .channels[TIM_5].channel = TIM_CH4,
		  .channels[TIM_5].duty = 0xffff>>1,
	#endif
	#else
	  .duty = 0xffff >> 1,
	#endif /* CONFIG_PWM_MULTICHAN */

	#  ifdef CONFIG_PWM_PULSECOUNT
	 .count = 0
	#  endif
	}
#endif
};


static struct motor_pwm_io_dev_s g_motor_pwm_io[CONFIG_MOTOR_LOWER_PWM_IO_NUMS] =
{
#if CONFIG_MOTOR_LOWER_PWM_IO_NUMS > 0
	{
		.pwm_dev   = NULL,
		.pwm_info  = &g_pwm_info[TIM_8],
		.gpiowrite = stm32_gpiowrite,

		#if CONFIG_PWM_NCHANNELS > 0
		.pinset[0] = GPIO_PWM8_IO_CH1,
		#endif

		#if CONFIG_PWM_NCHANNELS > 1
		.pinset[1] = GPIO_PWM8_IO_CH2,
		#endif

		#if CONFIG_PWM_NCHANNELS > 2
		.pinset[2] = GPIO_PWM8_IO_CH3,
		#endif

		#if CONFIG_PWM_NCHANNELS > 3
		.pinset[3] = GPIO_PWM8_IO_CH4,
		#endif
	}
	#endif
#if CONFIG_MOTOR_LOWER_PWM_IO_NUMS > 1
	,{
		.pwm_dev   = NULL,
		.pwm_info  = &g_pwm_info[TIM_5],
		.gpiowrite = stm32_gpiowrite,

		#if CONFIG_PWM_NCHANNELS > 0
		.pinset[0] = GPIO_PWM5_IO_CH1,
		#endif

		#if CONFIG_PWM_NCHANNELS > 1
		.pinset[1] = GPIO_PWM5_IO_CH2,
		#endif

		#if CONFIG_PWM_NCHANNELS > 2
		.pinset[2] = GPIO_PWM5_IO_CH3,
		#endif

		#if CONFIG_PWM_NCHANNELS > 3
		.pinset[3] = GPIO_PWM5_IO_CH4,
		#endif
	}
#endif
};

#endif /* defined(CONFIG_MOTOR_LOWER_PWM_IO) */
/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_elmo_db4x_setup
 *
 * Description:
 *  Initialize elmodb4x device
 *
 ************************************************************************************/

int stm32_elmo_db4x_setup(void)
{
	int ret;

#if defined(CONFIG_MOTOR_ELMO_DB4X)

#if defined(CONFIG_MOTOR_LOWER_CANBUS)
  struct motor_dev_s * elmo_can_dev[CONFIG_MOTOR_LOWER_CANBUS_NUMS];

  struct can_dev_s* can = NULL;

  for(int i = 0; i < CONFIG_MOTOR_LOWER_CANBUS_NUMS ; i++){
	  /* Initialize CANBUS motor devices */
	  elmo_can_dev[i] = elmodb4x_initialize(i+1,can);
	  if (elmo_can_dev[i] == NULL){
	      syslog(LOG_ERR,"ERROR:  Failed to get elmo_dev interface motor%d\n",i);
	      return -ENODEV;
	  }

	  /* Register the elmo_dev driver at "/dev/motorx" */
	  ret = motor_register(str_path[i], elmo_can_dev[i], elmodb4x_can_lower_init());
	  if (ret < 0){
	      syslog(LOG_ERR,"ERROR: motor%d register failed: %d\n",i , ret);
	      return ret;
	  }
  }
#else
  return -ENODEV;
#endif /* defined(CONFIG_MOTOR_LOWER_CANBUS) */


#if defined(CONFIG_MOTOR_LOWER_PWM_IO)

  struct motor_dev_s * elmo_pwm_dev[CONFIG_MOTOR_LOWER_PWM_IO_NUMS];

#if CONFIG_MOTOR_LOWER_PWM_IO_NUMS > 0
  g_motor_pwm_io[TIM_8].pwm_dev 	= stm32_pwminitialize(8);

  if (g_motor_pwm_io[TIM_8].pwm_dev == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM8 interface\n");
      return -ENODEV;
  }
#if CONFIG_PWM_NCHANNELS > 0
  stm32_configgpio(GPIO_PWM8_IO_CH1);
#endif
#if CONFIG_PWM_NCHANNELS > 1
  stm32_configgpio(GPIO_PWM8_IO_CH2);
#endif
#if CONFIG_PWM_NCHANNELS > 2
  stm32_configgpio(GPIO_PWM8_IO_CH3);
#endif
#if CONFIG_PWM_NCHANNELS > 3
  stm32_configgpio(GPIO_PWM8_IO_CH4);
#endif

#endif /* defined(CONFIG_MOTOR_LOWER_PWM_IO_NUMS) */

#if CONFIG_MOTOR_LOWER_PWM_IO_NUMS > 1
  g_motor_pwm_io[TIM_5].pwm_dev 	= stm32_pwminitialize(5);

  if (g_motor_pwm_io[TIM_5].pwm_dev == NULL){
      syslog(LOG_ERR,"ERROR:  Failed to get PWM8 interface\n");
      return -ENODEV;
  }

#if CONFIG_PWM_NCHANNELS > 0
  stm32_configgpio(GPIO_PWM5_IO_CH1);
#endif
#if CONFIG_PWM_NCHANNELS > 1
  stm32_configgpio(GPIO_PWM5_IO_CH2);
#endif
#if CONFIG_PWM_NCHANNELS > 2
  stm32_configgpio(GPIO_PWM5_IO_CH3);
#endif
#if CONFIG_PWM_NCHANNELS > 3
  stm32_configgpio(GPIO_PWM5_IO_CH4);
#endif
#endif /* defined(CONFIG_MOTOR_LOWER_PWM_IO_NUMS) */


  for(unsigned int i = 0; i < CONFIG_MOTOR_LOWER_PWM_IO_NUMS ; i++){
	  /* Initialize PWM motor devices */
	  elmo_pwm_dev[i] = elmo_db4x_pwm_io_initialize(i+1);
	  if (elmo_pwm_dev[i] == NULL){
	      syslog(LOG_ERR,"ERROR:  Failed to get elmo_dev interface motor_pwm%d\n",i+1);
	      return -ENODEV;
	  }

	  /* Register the elmo_dev driver at "/dev/motorx" */
	  ret = motor_register(pwm_str_path[i], elmo_pwm_dev[i], &g_motor_pwm_io[i]);
	  if (ret < 0){
	      syslog(LOG_ERR,"ERROR: motor_pwm%d register failed: %d\n",i , ret);
	      return ret;
	  }
  }

#else
  return -ENODEV;
#endif /* defined(CONFIG_MOTOR_LOWER_PWM_IO) */

  return OK;
#endif /* defined(CONFIG_MOTOR_ELMO_DB4X) */
}


