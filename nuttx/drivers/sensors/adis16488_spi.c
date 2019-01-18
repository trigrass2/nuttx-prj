/****************************************************************************
 * drivers/sensors/adis16488.c
 * Character driver for the ST ADIS16488 Tri-axis accelerometer and gyroscope.
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Bob Feretich <bob.feretich@rafresearch.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright+
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
 *****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_SPI) \
	&& defined(CONFIG_SPI_EXCHANGE) \
	&& defined(CONFIG_SENSORS_ADIS16488)

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>
#include <sys/types.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/sensors/adis16488.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: adis16488_read_register
 ****************************************************************************/

static uint16_t adis16488_spi_read_register(FAR struct adis16488_dev_s *dev,
                                     uint16_t reg_addr)
{
  uint16_t reg_data;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADIS16488_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, ADIS16488_SPI_MODE);

  /* Set CS to low to select the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to read. */

  reg_data = SPI_SEND(dev->spi, reg_addr);

  /* Write an idle byte while receiving the requested data */

  reg_data = (uint16_t) (SPI_SEND(dev->spi, 0xff));

  /* Set CS to high to deselect the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);

  return reg_data;
}

/******************************************************************************
 * Name: adis16488_read_registerblk
 ******************************************************************************/

 static void adis16488_spi_read_registerblk(FAR struct adis16488_dev_s *dev,
                                      uint8_t reg_addr,
                                      FAR uint8_t *reg_data,
                                      uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADIS16488_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, ADIS16488_SPI_MODE);

  /* Set CS to low to select the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address from where we want to start reading */

  SPI_SEND(dev->spi, reg_addr);

  /* Write idle bytes while receiving the requested data */

  while ( 0 != xfercnt-- )
    {
      *reg_data++ = (uint8_t)SPI_SEND(dev->spi, 0xff);
    }

  /* Set CS to high to deselect the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: adis16488_write_register
 ****************************************************************************/

static void adis16488_spi_write_register(FAR struct adis16488_dev_s *dev,
                                   uint8_t reg_addr, uint8_t reg_data)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADIS16488_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, ADIS16488_SPI_MODE);

  /* Set CS to low to select the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to write */

  SPI_SEND(dev->spi, reg_addr);

  /* Transmit the content which should be written into the register */

  SPI_SEND(dev->spi, reg_data);

  /* Set CS to high to deselect the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: adis16488_write_registerblk
 ****************************************************************************/

 static void adis16488_spi_write_registerblk(FAR struct adis16488_dev_s *dev,
                                       uint8_t reg_addr,
                                       FAR uint8_t *reg_data,
                                       uint8_t xfercnt)
{
  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(dev->spi, true);

  SPI_SETFREQUENCY(dev->spi, ADIS16488_SPI_FREQUENCY);

  SPI_SETMODE(dev->spi, ADIS16488_SPI_MODE);

  /* Set CS to low which selects the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, true);

  /* Transmit the register address to where we want to start writing */

  SPI_SEND(dev->spi, reg_addr );

  /* Transmit the content which should be written in the register block */

  while ( 0 != xfercnt-- )
    {
      SPI_SEND(dev->spi, *reg_data++);
    }

  /* Set CS to high to deselect the ADIS16488 */

  SPI_SELECT(dev->spi, dev->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(dev->spi, false);
}


/* *
 * operation function interface to hardware resource
 * SPI/IIC/CAN/UART.....
 * */

/**
 * adis_write_reg() - write 2 bytes from a 16-bit register
 * @dev: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value write to the device
 */
int adis_write_reg(struct adis16488_dev_s *dev, unsigned int reg,
	void* val, unsigned int size)
{
	uint32_t value = 0;

	unsigned int page = reg / ADIS_PAGE_SIZE;
	int ret;

	memcmp(&value,val,size);

	/* write page */
	if (dev->current_page != page) {
		dev->tx[0] = ADIS_WRITE_REG(ADIS_PAGE_SIZE);
		dev->tx[1] = page;
		dev->current_page = page;

		adis16488_spi_write_register(dev,dev->tx[0],dev->tx[1]);
	}

	/* write data to register */
	switch (size) {
	case 4:
		dev->tx[8] = ADIS_WRITE_REG(reg + 3);
		dev->tx[9] = (value >> 24) & 0xff;
		adis16488_spi_write_register(dev,dev->tx[8],dev->tx[9]);

		dev->tx[6] = ADIS_WRITE_REG(reg + 2);
		dev->tx[7] = (value >> 16) & 0xff;
		adis16488_spi_write_register(dev,dev->tx[6],dev->tx[7]);

	case 2:
		dev->tx[4] = ADIS_WRITE_REG(reg + 1);
		dev->tx[5] = (value >> 8) & 0xff;
		adis16488_spi_write_register(dev,dev->tx[4],dev->tx[5]);
	case 1:
		dev->tx[2] = ADIS_WRITE_REG(reg);
		dev->tx[3] = value & 0xff;
		adis16488_spi_write_register(dev,dev->tx[2],dev->tx[3]);

		ret = OK;
		break;

	default:
		ret = -EINVAL;
		return ret;
	}

	return ret;
}


/**
 * adis_read_reg() - read 2 bytes from a 16-bit register
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value read back from the device
 */
inline int adis_read_reg(struct adis16488_dev_s *dev, unsigned int reg,
	unsigned int *val, unsigned int size)
{
	uint32_t page = reg / ADIS_PAGE_SIZE;
	int ret;
	uint16_t regaddr,regval_low,regval_high;

	/* write page */
	if (dev->current_page != page) {
		dev->tx[0] = ADIS_WRITE_REG(ADIS_PAGE_SIZE);
		dev->tx[1] = page;
		dev->current_page = page;

		/* send message to device */
		adis16488_spi_write_register(dev,dev->tx[0],dev->tx[1]);

	}

	/* read data from register */
	switch (size) {
	case 4:
		dev->tx[2] = ADIS_READ_REG(reg + 2);
		dev->tx[3] = 0;
		regaddr = (dev->tx[2] << 8) + dev->tx[3];
		regval_high = adis16488_spi_read_register(dev,regaddr);
	case 2:
		dev->tx[4] = ADIS_READ_REG(reg);
		dev->tx[5] = 0;
		regaddr = (dev->tx[4] << 8) + dev->tx[5];
		regval_low = adis16488_spi_read_register(dev,regaddr);

		*val = (regval_high << 16) + regval_low;

		ret = OK;
		break;

	default:
		ret = -EINVAL;
		return ret;
	}

	return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_ADIS16488 && CONFIG_SPI_EXCHANGE */
