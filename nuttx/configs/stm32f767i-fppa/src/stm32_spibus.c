/************************************************************************************
 * configs/stm32f769i-disco/src/stm32_spibus.c
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

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <nuttx/spi/spi_transfer.h>
#include <nuttx/sensors/adis16488.h>

#include "up_arch.h"
#include "chip.h"
#include "stm32_spi.h"
#include "stm32_gpio.h"
#include "stm32f767i-fppa.h"

#if defined(CONFIG_STM32F7_SPI1) || defined(CONFIG_STM32F7_SPI2) || \
    defined(CONFIG_STM32F7_SPI3) || defined(CONFIG_STM32F7_SPI4) || \
    defined(CONFIG_STM32F7_SPI5)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f769i-disco board.
 *
 ************************************************************************************/

void weak_function stm32_spidev_initialize(void)
{
}

/****************************************************************************
 * Name:  stm32_spi1/2/3/4/5select and stm32_spi1/2/3/4/5status
 *
 * Description:
 *   The external functions, stm32_spi1/2/3select and stm32_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including stm32_spibus_initialize())
 *   are provided by common STM32 logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in stm32_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide stm32_spi1/2/3select() and stm32_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to stm32_spibus_initialize() in your low level application
 *      initialization logic
 *   4. The handle returned by stm32_spibus_initialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_SPI1
void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  if (devid == SPIDEV_USER(0))
    {
      stm32_gpiowrite(GPIO_SPI1_NSS, !selected);
    }
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32F7_SPI2
void stm32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  if (devid == SPIDEV_USER(0))
    {
      stm32_gpiowrite(GPIO_SPI2_NSS, !selected);
    }
}

uint8_t stm32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32F7_SPI3
void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
}

uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32F7_SPI4
void stm32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

}

uint8_t stm32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_STM32F7_SPI5
void stm32_spi5select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

}

uint8_t stm32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

/****************************************************************************
 * Name: stm32_spi1cmddata
 *
 * Description:
 *   Set or clear the SH1101A A0 or SD1306 D/C n bit to select data (true)
 *   or command (false). This function must be provided by platform-specific
 *   logic. This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
#ifdef CONFIG_STM32F7_SPI1
int stm32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32F7_SPI2
int stm32_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32F7_SPI3
int stm32_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32F7_SPI4
int stm32_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#ifdef CONFIG_STM32F7_SPI5
int stm32_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  return -ENODEV;
}
#endif

#endif /* CONFIG_SPI_CMDDATA */


/************************************************************************************
 * Name: stm32_spi_setup
 *
 * Description:
 *  Initialize SPI and register the SPI device
 *
 ************************************************************************************/

int stm32_spi_setup(void)
{
  struct spi_dev_s *spi;
  int ret , n_spi;
#ifdef CONFIG_STM32F7_SPI1
    /* Call stm32_spibus_initialize() to get an instance of the spi interface */
  	n_spi = 1;
	spi = stm32_spibus_initialize(n_spi);

  	if (spi == NULL){
      syslog(LOG_ERR,"[SPI]:  Failed to get spi%d interface\n",n_spi);
      return -ENODEV;
    }
  
	/* Register the spi driver at "/dev/spi1" */

	ret = spi_register(spi, n_spi);
	if (ret < 0){
		syslog(LOG_ERR,"ERROR: can_register failed: %d\n", ret);
      return ret;
    }
#endif	/*CONFIG_STM32F7_SPI1*/


#ifdef CONFIG_STM32F7_SPI2
    /* Call stm32_spibus_initialize() to get an instance of the spi interface */
	n_spi = 2;
	spi = stm32_spibus_initialize(n_spi);

  	if (spi == NULL){
      syslog(LOG_ERR,"[SPI]:  Failed to get spi%d interface\n",n_spi);
      return -ENODEV;
    }
  
#if defined(CONFIG_SENSORS_ADIS16488)
	/* Register the spi driver at "/dev/adis16488_1" */
	ret = adis16488_register(spi,1);
	if (ret < 0){
	  syslog(LOG_ERR,"[SPI]: Failed to register adis16488_1: %d\n", ret);
      return ret;
    }
#else
  	/*CONFIG_STM32F7_SPI5*/
	/* Register the spi driver at "/dev/spi5" */

	ret = spi_register(spi, n_spi);
	if (ret < 0){
	  syslog(LOG_ERR,"[SPI]: Failed to register spi%d: %d\n", n_spi, ret);
      return ret;
    }
#endif	/*CONFIG_SENSORS_ADIS16488*/
#endif	/*CONFIG_STM32F7_SPI2*/


#ifdef CONFIG_STM32F7_SPI3
    /* Call stm32_spibus_initialize() to get an instance of the spi interface */
	n_spi = 3;
	spi = stm32_spibus_initialize(n_spi);

  	if (spi == NULL){
      syslog(LOG_ERR,"[SPI]:  Failed to get spi%d interface\n",n_spi);
      return -ENODEV;
    }
  
	/* Register the spi driver at "/dev/spi3" */

	ret = spi_register(spi, n_spi);
	if (ret < 0){
	  syslog(LOG_ERR,"ERROR: can_register failed: %d\n", ret);
      return ret;
    }
#endif	/*CONFIG_STM32F7_SPI3*/


#ifdef CONFIG_STM32F7_SPI4
    /* Call stm32_spibus_initialize() to get an instance of the spi interface */
	n_spi = 4;
	spi = stm32_spibus_initialize(n_spi);

  	if (spi == NULL){
      		syslog(LOG_ERR,"[SPI]:  Failed to get spi%d interface\n",n_spi);
      return -ENODEV;
    }
  
	/* Register the spi driver at "/dev/spi4" */

	ret = spi_register(spi, n_spi);
	if (ret < 0){
	  syslog(LOG_ERR,"ERROR: can_register failed: %d\n", ret);
      return ret;
    }
#endif	/*CONFIG_STM32F7_SPI4*/


#ifdef CONFIG_STM32F7_SPI5
    /* Call stm32_spibus_initialize() to get an instance of the spi interface */
	n_spi = 5;
	spi = stm32_spibus_initialize(n_spi);

  	if (spi == NULL){
          syslog(LOG_ERR,"[SPI]:  Failed to get spi%d interface\n",n_spi);
      return -ENODEV;
    }

  	/*CONFIG_STM32F7_SPI5*/
	/* Register the spi driver at "/dev/spi5" */
	ret = spi_register(spi, n_spi);
	if (ret < 0){
	   syslog(LOG_ERR,"ERROR: can_register failed: %d\n", ret);
      return ret;
    }

#endif	/*CONFIG_STM32F7_SPI5*/

#if defined (CONFIG_STM32F7_SPI1) || (CONFIG_STM32F7_SPI2) || (CONFIG_STM32F7_SPI3) || \
			(CONFIG_STM32F7_SPI4) || (CONFIG_STM32F7_SPI5)

  return OK;

#else

  return -ENODEV;

#endif

}




#endif /* CONFIG_STM32F7_SPI1 || ... CONFIG_STM32F7_SPI5 */
