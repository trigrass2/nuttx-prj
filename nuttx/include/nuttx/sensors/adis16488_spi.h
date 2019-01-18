/******************************************************************************
 * include/nuttx/sensors/adis16488.h
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Bob Feretich <bob.feretich@rafresearch.com>
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
 ******************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_ADIS16488_H
#define __INCLUDE_NUTTX_SENSORS_ADIS16488_H

/******************************************************************************
 * Driver usage notes:
 *
 * This driver is a "kernel sensor leaf driver" that may be used directly from
 * user applications via the file_operations interface or have selected entry
 * points called directly from a "kernel sensor cluster driver".
 *
 * To use this driver via the file_operations interface, the board
 * initialization function should call this driver's registration function.
 * The driver will register itself with Nuttx under the /dev path that is
 * provided by the config structure.  Then user applications may access the
 * driver via the "file descriptor handle" returned by the file_operations
 * open() function.
 *
 * By default the open() function configures the sensor for:
 *
 *   Output Data Rate (ODR) = 1600 Hz.
 *   Bandwidth (BW) = 800 Hz.
 *   Normal mode sampling (as opposed to low power mode sampling).
 *   The Low Pass Filter is enabled and the High Pass Filter is disabled.
 *   A filter settling time of 370ms is selected.
 *
 * If the user desires a different configuration settings, the the user may
 * either provide a pointer to an array of "struct adis16488_reg_pair_s" that
 * will be applied to to the sensor upon open(); or dynamically use
 * the lseek() and write() file_operations functions to set the
 * sensor configuration as desired.
 *
 * When using the sensor from the file_operations interface, the sensor is
 * accessed in Programmed I/O (PIO) mode. (i.e. When the read() function is
 * executed, the sensor is read on that thread.) PIO reads and writes block
 * the calling thread until data is available. Since the sensor is on an SPI
 * bus running at near 10 MHz, the read or write operations should only take
 * a few microseconds (about a microsecond per byte of data), so for
 * individual sensor reads and writes, the overhead of using interrupts or
 * DMA is not worthwhile.
 *
 * This driver supports the Common Sensor Register Interface.
 * See drivers/sensors/README.txt for details.
 *
 * This driver supports the Sensor Cluster Driver Interface.
 * See drivers/sensors/README.txt for details.
 *
 * It also extends the interface by permitting cluster driver calls to
 * a function that is intended to perform high performance DMA SPI exchange
 * operations. See the usage note on the exchange operation below.
 *
 ****************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************
 */

#include <semaphore.h>
#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/cluster_driver.h>

#if defined(CONFIG_SPI) \
	&& defined(CONFIG_SPI_EXCHANGE) \
	&& defined(CONFIG_SENSORS_ADIS16488)

/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************
 */

/****************************************************************************
 * Name: adis_write_reg
 ****************************************************************************/

int adis_write_reg(struct adis16488_dev_s *dev, unsigned int reg,\
	void *val, unsigned int size);

/****************************************************************************
* Name: adis_read_reg
****************************************************************************/

int adis_read_reg(struct adis16488_dev_s *dev, unsigned int reg,\
				  unsigned int *val, unsigned int size);


/****************************************************************************
* Name: adis16488_spi_read_registerblk
****************************************************************************/

static uint16_t adis16488_spi_read_register(FAR struct adis16488_dev_s *dev,
                                    uint16_t reg_addr);
/****************************************************************************
 * Name: adis16488_spi_read_registerblk
 ****************************************************************************/

static void     adis16488_spi_read_registerblk(FAR struct adis16488_dev_s *dev,
                                    uint8_t reg_addr,
                                    FAR uint8_t *reg_data,
                                    uint8_t xfercnt);
/****************************************************************************
 * Name: adis16488_spi_write_register
 ****************************************************************************/

static void     adis16488_spi_write_register(FAR struct adis16488_dev_s *dev,
                                    uint8_t reg_addr,
                                    uint8_t reg_data);

/****************************************************************************
 * Name: adis16488_spi_write_registerblk
 ****************************************************************************/

static void     adis16488_spi_write_registerblk(FAR struct adis16488_dev_s *dev,
                                    uint8_t reg_addr,
                                    FAR uint8_t *reg_data,
                                    uint8_t xfercnt);



#endif /* CONFIG_SPI && CONFIG_SENSORS_ADIS16488 && CONFIG_SPI_EXCHANGE */
#endif /* __INCLUDE_NUTTX_SENSORS_ADIS16488_H */
