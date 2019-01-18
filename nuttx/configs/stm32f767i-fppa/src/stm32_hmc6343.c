/************************************************************************************
 * configs/stm32f769i-disco/src/stm32_hmc6343.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>
#include <nuttx/sensors/hmc6343.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_i2c.h"
#include "stm32_gpio.h"
#include "stm32f767i-fppa.h"

#if defined(CONFIG_STM32F7_I2C1) || defined(CONFIG_STM32F7_I2C2) || \
    defined(CONFIG_STM32F7_I2C3) || defined(CONFIG_STM32F7_I2C4)

enum {
	I2C1 = 1,
	I2C2,
	I2C3,
	I2C4
};
/************************************************************************************
 * Public Functions
 ************************************************************************************/


/************************************************************************************
 * Name: stm32_hmc6343_setup
 *
 * Description:
 *  Initialize I2C and register the HMC5343 device
 *
 ************************************************************************************/

int stm32_hmc6343_setup(void)
{
  struct i2c_master_s *i2c_master;
  int ret = 0, n_i2c = 0;

#ifdef CONFIG_STM32F7_I2C1
    /* Call stm32_i2cbus_initialize() to get an instance of the i2c_master interface */
  	n_i2c ++;
	i2c_master = stm32_i2cbus_initialize(I2C1);

  	if (i2c_master == NULL){
      snerr("failed to get i2c_%d interface\n",I2C1);
      return -ENODEV;
    }
  
	/* Register the hmc6343 driver at "/dev/hmc6343_x" */

	ret = hmc6343_register(i2c_master, n_i2c);
	if (ret < 0){
		  snerr("hmc6343_register failed: %d\n", ret);
      return ret;
    }
#endif	/*CONFIG_STM32F7_I2C1*/

#ifdef CONFIG_STM32F7_I2C2
    /* Call stm32_i2cbus_initialize() to get an instance of the i2c_master interface */
    n_i2c++;
    i2c_master = stm32_i2cbus_initialize(I2C2);

    if (i2c_master == NULL){
      snerr("failed to get i2c_%d interface\n",I2C2);
      return -ENODEV;
    }
  
  /* Register the hmc6343 driver at "/dev/hmc6343_x" */

  ret = hmc6343_register(i2c_master, n_i2c);
  if (ret < 0){
      snerr("hmc6343_register failed: %d\n", ret);
      return ret;
    }
#endif  /*CONFIG_STM32F7_I2C2*/

#ifdef CONFIG_STM32F7_I2C3
    /* Call stm32_i2cbus_initialize() to get an instance of the i2c_master interface */
    n_i2c++;
    i2c_master = stm32_i2cbus_initialize(I2C3);

    if (i2c_master == NULL){
      snerr("failed to get i2c_%d interface\n",I2C3);
      return -ENODEV;
    }
  
  /* Register the hmc6343 driver at "/dev/hmc6343_x" */

  ret = hmc6343_register(i2c_master, n_i2c);
  if (ret < 0){
      snerr("hmc6343_register failed: %d\n", ret);
      return ret;
    }
#endif  /*CONFIG_STM32F7_I2C3*/

#ifdef CONFIG_STM32F7_I2C4
    /* Call stm32_i2cbus_initialize() to get an instance of the i2c_master interface */
    n_i2c++;
    i2c_master = stm32_i2cbus_initialize(I2C4);

    if (i2c_master == NULL){
      snerr("failed to get i2c_%d interface\n",I2C3);
      return -ENODEV;
    }
  
  /* Register the hmc6343 driver at "/dev/hmc6343_x" */

  ret = hmc6343_register(i2c_master, n_i2c);
  if (ret < 0){
      snerr("hmc6343_register failed: %d\n", ret);
      return ret;
    }
#endif  /*CONFIG_STM32F7_I2C4*/

  return ret;
}

#endif /* CONFIG_STM32F7_I2C1 || ... CONFIG_STM32F7_I2C4 */


