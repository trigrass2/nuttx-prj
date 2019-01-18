/************************************************************************************
 * configs/stm32f769i-disco/src/stm32_n25qxxx.c
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

#include <arch/board/board.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_qspi.h"
#include "stm32_gpio.h"


/************************************************************************************
 * Public Functions
 ************************************************************************************/


/************************************************************************************
 * Name: stm32_n25qxxx_setup
 *
 * Description:
 *  Initialize QUADSPI and register the N25QXXX device
 *
 ************************************************************************************/

int stm32_n25qxxx_setup(void)
{

	int ret = 0;

#ifdef CONFIG_STM32F7_QUADSPI

	struct qspi_dev_s *qspi_dev;

	/* Call stm32_qspi_initialize() to get an instance of the qspi_dev interface */
	qspi_dev = stm32_qspi_initialize(0);

#endif  /*CONFIG_STM32F7_QUADSPI*/

#ifdef CONFIG_MTD_N25QXXX
	struct mtd_dev_s *n25qxxx;

	/* Register the n25qxxx driver */
	n25qxxx = n25qxxx_initialize(qspi_dev,true);

	/* check device register status */
	if(n25qxxx == NULL){
		ret = -1;
	}

#endif  /*CONFIG_MTD_N25QXXX*/

  return ret;
}



