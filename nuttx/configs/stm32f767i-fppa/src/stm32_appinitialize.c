/****************************************************************************
 * config/stm32f769i-disco/src/stm32_appinitialize.c
 *
 *   Copyright (C) 2015-2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <sys/mount.h>
#include <sys/types.h>
#include <syslog.h>
#include <debug.h>
//#include "stm32_ccm.h"
#include "stm32f767i-fppa.h"
#include <nuttx/timers/drv_hrt.h>
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{

#ifdef CONFIG_FS_PROCFS
  int ret;

#ifdef CONFIG_STM32_CCM_PROCFS
  /* Register the CCM procfs entry.  This must be done before the procfs is
   * mounted.
   */

  (void)ccm_procfs_register();
#endif

  /* Mount the procfs file system */
  ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0){
	syslog(LOG_ERR, "[PROCFS]: Failed to mount procfs at %s: %d\n","/proc", ret);
    }
#endif

#ifdef CONFIG_TIMER
  /* Mount timer device */
  ret = stm32_timer_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[TIMER]: Failed to setup timer: %d\n", ret);
  }
	/* configure the high-resolution time/callout interface */
	hrt_init();
#endif

#ifdef CONFIG_SPI
  /* Mount spi device */
  ret = stm32_spi_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[SPI]: Failed to setup spi: %d\n", ret);
  }
#endif

#if (defined CONFIG_PWM) && (!defined CONFIG_MOTOR_LOWER_PWM_IO)
  /* Mount pwm device */
  ret = stm32_pwm_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[PWM]: Failed to setup pwm: %d\n", ret);
  }
#endif

#if (defined CONFIG_MOTOR_ELMO_DB4X)
  /* Mount elmo device */
  ret = stm32_elmo_db4x_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[ELMO]: Failed to setup elmo: %d\n", ret);
  }
#endif

#ifdef CONFIG_CAN
  /* Mount can device */
  ret = stm32_can_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[CAN]: Failed to setup can: %d\n", ret);
  }
#endif

#ifdef CONFIG_HALLS
  ret = stm32_hall_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[HALL]: Failed to setup hall 1: %d\n", ret);
  }
#endif

#ifdef CONFIG_MMCSD
  /* Initialize the SDIO block driver */
//  ret = OK;
//  ret = stm32_sdio_initialize();
//  if (ret != OK)
//    {
//      ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
//    }
#endif

#ifdef CONFIG_ADC
  ret = stm32_adc_manager_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[ADC]: Failed to setup adc manager: %d\n", ret);
  }

#endif

#if (defined CONFIG_I2C) && (defined CONFIG_SENSORS_HMC6343)
  /* Mount HMC6343 device */
  ret = stm32_hmc6343_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[MAG]: Failed to setup hmc6343: %d\n", ret);
  }
#endif

#if (defined CONFIG_I2C) && (defined CONFIG_I2C_EE_24XX)
  /* Mount EEPROM device */
  ret = stm32_i2c_xx24xx_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[EEPROM]: Failed to setup eeprom xx24xx: %d\n", ret);
  }
#endif


#if (defined CONFIG_STM32F7_QUADSPI) && (defined CONFIG_MTD_N25QXXX)
  /* Mount QUADSPI flash device */
  ret = stm32_n25qxxx_setup();
  if(ret < 0 ){
	syslog(LOG_ERR, "[EEPROM]: Failed to setup qspi-flash n25qxxx: %d\n", ret);
  }
#endif

	board_buzzer_initialize();

  return OK;
}
