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

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <semaphore.h>
#include <nuttx/irq.h>
#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/cluster_driver.h>

#include "../../../../apps/include/uORB/uorb/uORB.h"
#include "../../../../apps/include/uORB/topic/adis16488_uorb.h"


#define DEG2RAD				0.017453293
#define RAD2DEG				57.29578

#if defined(CONFIG_SPI) \
	&& defined(CONFIG_SPI_EXCHANGE) \
	&& defined(CONFIG_SENSORS_ADIS16488)

#define ADISIOC_MEASUREMENT_RATE_SET		_SNIOC(1)
#define ADISIOC_MEASUREMENT_START			_SNIOC(2)
#define ADISIOC_MEASUREMENT_STOP			_SNIOC(3)
/*******************************************************************************
 * Pre-processor Definitions
 *******************************************************************************
 */

#define BIT(nr)					(1 << (nr))
#define ADIS_PAGE_SIZE 			0x80
#define ADIS_REG_PAGE_ID 		0x00
#define ADIS_WRITE_REG(reg) 	((0x80 | (reg)))
#define ADIS_READ_REG(reg) 		((reg) & 0x7f)

#define ADIS16488_REG(page, reg) ((page) * ADIS_PAGE_SIZE + (reg))

#define ADIS16488_REG_PAGE_ID 0x00 /* Same address on each page */
#define ADIS16488_REG_SEQ_CNT			ADIS16488_REG(0x00, 0x06)
#define ADIS16488_REG_SYS_E_FLA			ADIS16488_REG(0x00, 0x08)
#define ADIS16488_REG_DIAG_STS			ADIS16488_REG(0x00, 0x0A)
#define ADIS16488_REG_ALM_STS			ADIS16488_REG(0x00, 0x0C)
#define ADIS16488_REG_TEMP_OUT			ADIS16488_REG(0x00, 0x0E)
#define ADIS16488_REG_X_GYRO_OUT		ADIS16488_REG(0x00, 0x10)
#define ADIS16488_REG_Y_GYRO_OUT		ADIS16488_REG(0x00, 0x14)
#define ADIS16488_REG_Z_GYRO_OUT		ADIS16488_REG(0x00, 0x18)
#define ADIS16488_REG_X_ACCEL_OUT		ADIS16488_REG(0x00, 0x1C)
#define ADIS16488_REG_Y_ACCEL_OUT		ADIS16488_REG(0x00, 0x20)
#define ADIS16488_REG_Z_ACCEL_OUT		ADIS16488_REG(0x00, 0x24)
#define ADIS16488_REG_X_MAGN_OUT		ADIS16488_REG(0x00, 0x28)
#define ADIS16488_REG_Y_MAGN_OUT		ADIS16488_REG(0x00, 0x2A)
#define ADIS16488_REG_Z_MAGN_OUT		ADIS16488_REG(0x00, 0x2C)
#define ADIS16488_REG_BAROM_OUT			ADIS16488_REG(0x00, 0x2E)
#define ADIS16488_REG_X_DELTAANG_OUT		ADIS16488_REG(0x00, 0x40)
#define ADIS16488_REG_Y_DELTAANG_OUT		ADIS16488_REG(0x00, 0x44)
#define ADIS16488_REG_Z_DELTAANG_OUT		ADIS16488_REG(0x00, 0x48)
#define ADIS16488_REG_X_DELTAVEL_OUT		ADIS16488_REG(0x00, 0x4C)
#define ADIS16488_REG_Y_DELTAVEL_OUT		ADIS16488_REG(0x00, 0x50)
#define ADIS16488_REG_Z_DELTAVEL_OUT		ADIS16488_REG(0x00, 0x54)
#define ADIS16488_REG_PROD_ID			ADIS16488_REG(0x00, 0x7E)

#define ADIS16488_REG_X_GYRO_SCALE		ADIS16488_REG(0x02, 0x04)
#define ADIS16488_REG_Y_GYRO_SCALE		ADIS16488_REG(0x02, 0x06)
#define ADIS16488_REG_Z_GYRO_SCALE		ADIS16488_REG(0x02, 0x08)
#define ADIS16488_REG_X_ACCEL_SCALE		ADIS16488_REG(0x02, 0x0A)
#define ADIS16488_REG_Y_ACCEL_SCALE		ADIS16488_REG(0x02, 0x0C)
#define ADIS16488_REG_Z_ACCEL_SCALE		ADIS16488_REG(0x02, 0x0E)
#define ADIS16488_REG_X_GYRO_BIAS		ADIS16488_REG(0x02, 0x10)
#define ADIS16488_REG_Y_GYRO_BIAS		ADIS16488_REG(0x02, 0x14)
#define ADIS16488_REG_Z_GYRO_BIAS		ADIS16488_REG(0x02, 0x18)
#define ADIS16488_REG_X_ACCEL_BIAS		ADIS16488_REG(0x02, 0x1C)
#define ADIS16488_REG_Y_ACCEL_BIAS		ADIS16488_REG(0x02, 0x20)
#define ADIS16488_REG_Z_ACCEL_BIAS		ADIS16488_REG(0x02, 0x24)
#define ADIS16488_REG_X_HARD_IRON		ADIS16488_REG(0x02, 0x28)
#define ADIS16488_REG_Y_HARD_IRON		ADIS16488_REG(0x02, 0x2A)
#define ADIS16488_REG_Z_HARD_IRON		ADIS16488_REG(0x02, 0x2C)
#define ADIS16488_REG_BAROM_BIAS		ADIS16488_REG(0x02, 0x40)
#define ADIS16488_REG_FLASH_CNT			ADIS16488_REG(0x02, 0x7C)

#define ADIS16488_REG_GLOB_CMD			ADIS16488_REG(0x03, 0x02)
#define ADIS16488_REG_FNCTIO_CTRL		ADIS16488_REG(0x03, 0x06)
#define ADIS16488_REG_GPIO_CTRL			ADIS16488_REG(0x03, 0x08)
#define ADIS16488_REG_CONFIG			ADIS16488_REG(0x03, 0x0A)
#define ADIS16488_REG_DEC_RATE			ADIS16488_REG(0x03, 0x0C)
#define ADIS16488_REG_SLP_CNT			ADIS16488_REG(0x03, 0x10)
#define ADIS16488_REG_FILTER_BNK0		ADIS16488_REG(0x03, 0x16)
#define ADIS16488_REG_FILTER_BNK1		ADIS16488_REG(0x03, 0x18)
#define ADIS16488_REG_ALM_CNFG0			ADIS16488_REG(0x03, 0x20)
#define ADIS16488_REG_ALM_CNFG1			ADIS16488_REG(0x03, 0x22)
#define ADIS16488_REG_ALM_CNFG2			ADIS16488_REG(0x03, 0x24)
#define ADIS16488_REG_XG_ALM_MAGN		ADIS16488_REG(0x03, 0x28)
#define ADIS16488_REG_YG_ALM_MAGN		ADIS16488_REG(0x03, 0x2A)
#define ADIS16488_REG_ZG_ALM_MAGN		ADIS16488_REG(0x03, 0x2C)
#define ADIS16488_REG_XA_ALM_MAGN		ADIS16488_REG(0x03, 0x2E)
#define ADIS16488_REG_YA_ALM_MAGN		ADIS16488_REG(0x03, 0x30)
#define ADIS16488_REG_ZA_ALM_MAGN		ADIS16488_REG(0x03, 0x32)
#define ADIS16488_REG_XM_ALM_MAGN		ADIS16488_REG(0x03, 0x34)
#define ADIS16488_REG_YM_ALM_MAGN		ADIS16488_REG(0x03, 0x36)
#define ADIS16488_REG_ZM_ALM_MAGN		ADIS16488_REG(0x03, 0x38)
#define ADIS16488_REG_BR_ALM_MAGN		ADIS16488_REG(0x03, 0x3A)
#define ADIS16488_REG_FIRM_REV			ADIS16488_REG(0x03, 0x78)
#define ADIS16488_REG_FIRM_DM			ADIS16488_REG(0x03, 0x7A)
#define ADIS16488_REG_FIRM_Y			ADIS16488_REG(0x03, 0x7C)

#define ADIS16488_REG_SERIAL_NUM		ADIS16488_REG(0x04, 0x20)


/* Each filter coefficent bank spans two pages */
#define ADIS16488_FIR_COEF(page, x) (x < 60 ? ADIS16488_REG(page, (x) + 8) : \
		ADIS16488_REG((page) + 1, (x) - 60 + 8))
#define ADIS16488_FIR_COEF_A(x)			ADIS16488_FIR_COEF(0x05, (x))
#define ADIS16488_FIR_COEF_B(x)			ADIS16488_FIR_COEF(0x07, (x))
#define ADIS16488_FIR_COEF_C(x)			ADIS16488_FIR_COEF(0x09, (x))
#define ADIS16488_FIR_COEF_D(x)			ADIS16488_FIR_COEF(0x0B, (x))

/*
 * Configure parameters
 */

#define ADIS16488_STARTUP_DELAY	290 /* ms */
#define ADIS16488_MTEST_DELAY 90 /* ms */

#define ADIS_MSC_CTRL_DATA_RDY_EN		BIT(2)
#define ADIS_MSC_CTRL_DATA_RDY_POL_HIGH	BIT(1)
#define ADIS_MSC_CTRL_DATA_RDY_DIO2		BIT(0)


#define ADIS16488_FLASH_CNT  0x00 /* Flash memory write count */
#define ADIS16488_SUPPLY_OUT 0x02 /* Power supply measurement */
#define ADIS16488_XGYRO_OUT  0x04 /* X-axis gyroscope output */
#define ADIS16488_YGYRO_OUT  0x06 /* Y-axis gyroscope output */
#define ADIS16488_ZGYRO_OUT  0x08 /* Z-axis gyroscope output */
#define ADIS16488_XACCL_OUT  0x0A /* X-axis accelerometer output */
#define ADIS16488_YACCL_OUT  0x0C /* Y-axis accelerometer output */
#define ADIS16488_ZACCL_OUT  0x0E /* Z-axis accelerometer output */
#define ADIS16488_XMAGN_OUT  0x10 /* X-axis magnetometer measurement */
#define ADIS16488_YMAGN_OUT  0x12 /* Y-axis magnetometer measurement */
#define ADIS16488_ZMAGN_OUT  0x14 /* Z-axis magnetometer measurement */
#define ADIS16488_TEMP_OUT   0x16 /* Temperature output */
#define ADIS16488_AUX_ADC    0x18 /* Auxiliary ADC measurement */

#define ADIS16488_XTEMP_OUT  0x10 /* X-axis gyroscope temperature measurement */
#define ADIS16488_YTEMP_OUT  0x12 /* Y-axis gyroscope temperature measurement */
#define ADIS16488_ZTEMP_OUT  0x14 /* Z-axis gyroscope temperature measurement */

#define ADIS16300_PITCH_OUT  0x12 /* X axis inclinometer output measurement */
#define ADIS16300_ROLL_OUT   0x14 /* Y axis inclinometer output measurement */
#define ADIS16300_AUX_ADC    0x16 /* Auxiliary ADC measurement */

#define ADIS16448_BARO_OUT	 0x16 /* Barometric pressure output */
#define ADIS16448_TEMP_OUT   0x18 /* Temperature output */

/* Calibration parameters */
#define ADIS16488_XGYRO_OFF 0x1A /* X-axis gyroscope bias offset factor */
#define ADIS16488_YGYRO_OFF 0x1C /* Y-axis gyroscope bias offset factor */
#define ADIS16488_ZGYRO_OFF 0x1E /* Z-axis gyroscope bias offset factor */
#define ADIS16488_XACCL_OFF 0x20 /* X-axis acceleration bias offset factor */
#define ADIS16488_YACCL_OFF 0x22 /* Y-axis acceleration bias offset factor */
#define ADIS16488_ZACCL_OFF 0x24 /* Z-axis acceleration bias offset factor */
#define ADIS16488_XMAGN_HIF 0x26 /* X-axis magnetometer, hard-iron factor */
#define ADIS16488_YMAGN_HIF 0x28 /* Y-axis magnetometer, hard-iron factor */
#define ADIS16488_ZMAGN_HIF 0x2A /* Z-axis magnetometer, hard-iron factor */
#define ADIS16488_XMAGN_SIF 0x2C /* X-axis magnetometer, soft-iron factor */
#define ADIS16488_YMAGN_SIF 0x2E /* Y-axis magnetometer, soft-iron factor */
#define ADIS16488_ZMAGN_SIF 0x30 /* Z-axis magnetometer, soft-iron factor */

#define ADIS16488_GPIO_CTRL 0x32 /* Auxiliary digital input/output control */
#define ADIS16488_MSC_CTRL  0x34 /* Miscellaneous control */
#define ADIS16488_SMPL_PRD  0x36 /* Internal sample period (rate) control */
#define ADIS16488_SENS_AVG  0x38 /* Dynamic range and digital filter control */
#define ADIS16488_SLP_CNT   0x3A /* Sleep mode control */
#define ADIS16488_DIAG_STAT 0x3C /* System status */

/* Alarm functions */
#define ADIS16488_GLOB_CMD  0x3E /* System command */
#define ADIS16488_ALM_MAG1  0x40 /* Alarm 1 amplitude threshold */
#define ADIS16488_ALM_MAG2  0x42 /* Alarm 2 amplitude threshold */
#define ADIS16488_ALM_SMPL1 0x44 /* Alarm 1 sample size */
#define ADIS16488_ALM_SMPL2 0x46 /* Alarm 2 sample size */
#define ADIS16488_ALM_CTRL  0x48 /* Alarm control */
#define ADIS16488_AUX_DAC   0x4A /* Auxiliary DAC data */

#define ADIS16334_LOT_ID1   	0x52 /* Lot identification code 1 */
#define ADIS16334_LOT_ID2   	0x54 /* Lot identification code 2 */
#define ADIS16488_PRODUCT_ID 	0x56 /* Product identifier */
#define ADIS16334_SERIAL_NUMBER 0x58 /* Serial number, lot specific */

#define ADIS16488_ERROR_ACTIVE		(1<<14)
#define ADIS16488_NEW_DATA			(1<<14)

/* MSC_CTRL */
#define ADIS16488_MSC_CTRL_MEM_TEST				(1<<11)
#define ADIS16488_MSC_CTRL_INT_SELF_TEST		(1<<10)
#define ADIS16488_MSC_CTRL_NEG_SELF_TEST		(1<<9)
#define ADIS16488_MSC_CTRL_POS_SELF_TEST		(1<<8)
#define ADIS16488_MSC_CTRL_GYRO_BIAS			(1<<7)
#define ADIS16488_MSC_CTRL_ACCL_ALIGN			(1<<6)
#define ADIS16488_MSC_CTRL_DATA_RDY_EN			(1<<2)
#define ADIS16488_MSC_CTRL_DATA_RDY_POL_HIGH	(1<<1)
#define ADIS16488_MSC_CTRL_DATA_RDY_DIO2		(1<<0)

/* SMPL_PRD */
#define ADIS16488_SMPL_PRD_TIME_BASE	(1<<7)
#define ADIS16488_SMPL_PRD_DIV_MASK		0x7F

/* DIAG_STAT */
#define ADIS16488_DIAG_STAT_ZACCL_FAIL	15
#define ADIS16488_DIAG_STAT_YACCL_FAIL	14
#define ADIS16488_DIAG_STAT_XACCL_FAIL	13
#define ADIS16488_DIAG_STAT_XGYRO_FAIL	12
#define ADIS16488_DIAG_STAT_YGYRO_FAIL	11
#define ADIS16488_DIAG_STAT_ZGYRO_FAIL	10
#define ADIS16488_DIAG_STAT_ALARM2		9
#define ADIS16488_DIAG_STAT_ALARM1		8
#define ADIS16488_DIAG_STAT_FLASH_CHK	6
#define ADIS16488_DIAG_STAT_SELF_TEST	5
#define ADIS16488_DIAG_STAT_OVERFLOW	4
#define ADIS16488_DIAG_STAT_SPI_FAIL	3
#define ADIS16488_DIAG_STAT_FLASH_UPT	2
#define ADIS16488_DIAG_STAT_POWER_HIGH	1
#define ADIS16488_DIAG_STAT_POWER_LOW	0

/* GLOB_CMD */
/* Global commands register */
#define ADIS_GLOB_CMD_BIAS_NULL							BIT(0)
#define ADIS_GLOB_CMD_SELF_TEST							BIT(1)
#define ADIS_GLOB_CMD_FLASH_TEST						BIT(2)
#define ADIS_GLOB_CMD_FLASH_UPDATE						BIT(3)
#define ADIS_GLOB_CMD_FACTORY_CALIBRATION_RESTORE		BIT(6)
#define ADIS_GLOB_CMD_SOFTWARE_RESET					BIT(7)

/* SLP_CNT */
#define ADIS16488_SLP_CNT_POWER_OFF		(1<<8)

#define ADIS16334_RATE_DIV_SHIFT 		8
#define ADIS16334_RATE_INT_CLK 			BIT(0)

#define ADIS16488_SPI_SLOW				(uint32_t)(300000)
#define ADIS16488_SPI_NORMAL			(uint32_t)(1000000)
#define ADIS16488_SPI_FAST				(uint32_t)(2000000)
#define ADIS16488_SPI_BURST				(uint32_t)(4000000)

#define ADIS16488_HAS_PROD_ID		BIT(0)
#define ADIS16488_NO_BURST			BIT(1)
#define ADIS16488_HAS_SLOW_MODE		BIT(2)
#define ADIS16488_HAS_SERIAL_NUMBER	BIT(3)
#define ADIS16488_BURST_DIAG_STAT	BIT(4)



/* SPI Bus Parameters */

#define ADIS16488_SPI_FREQUENCY           (ADIS16488_SPI_BURST)   /* 4 MHz */
#define ADIS16488_SPI_MODE                (SPIDEV_MODE3) 		  /* SPI Mode 3: CPOL=1,CPHA=1 */

/****************************************************************************
 * Private structure definitions
 ****************************************************************************/
enum
{
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	MAX_AXIS,
};

struct sensor_data_s
{
  float gyro[MAX_AXIS];                      /* Measurement accelerometer */
  float acc[MAX_AXIS];                       /* Measurement gyroscale */
  float mag[MAX_AXIS];                       /* Measurement magnetometer */
  float delta_v[MAX_AXIS];                   /* Measurement delta_vel */
  float angle[MAX_AXIS];                     /* Measurement angle */
  float alt;                         		   /* Measurement altitude */
  float temp;                        		   /* Measurement temperature */
};

 /**
  * struct adis_data - ADIS chip variant specific data
  * @read_delay: SPI delay for read operations in us
  * @write_delay: SPI delay for write operations in us
  * @glob_cmd_reg: Register address of the GLOB_CMD register
  * @msc_ctrl_reg: Register address of the MSC_CTRL register
  * @diag_stat_reg: Register address of the DIAG_STAT register
  * @status_error_msgs: Array of error messgaes
  * @status_error_mask:
  */
 struct adis_data {
 	unsigned int read_delay;
 	unsigned int write_delay;

 	unsigned int glob_cmd_reg;
 	unsigned int msc_ctrl_reg;
 	unsigned int diag_stat_reg;

 	unsigned int self_test_mask;
 	bool self_test_no_autoclear;
 	unsigned int startup_delay;

 	const char * const *status_error_msgs;
 	unsigned int status_error_mask;

 	bool has_paging;
 };

struct adis16488_dev_s
{
  FAR struct spi_dev_s *spi;             /* Pointer to the SPI instance */
  FAR struct adis16488_config_s *config; /* Pointer to the configuration of the
                                        * ADIS16488 sensor */
  sem_t devicesem;                     /* Manages exclusive access to this
                                        * device */
  sem_t datasem;                       /* Manages exclusive access to this
                                        * structure */
  struct sensor_data_s sdata;          /* The data as measured by the sensor */
  struct adis_data data;
  uint8_t cd_ocount;        		   /* The number of times the device has been opened */
  uint16_t seek_address;               /* Current device address. */
  uint8_t readonly;                    /* 0 = writing to the device in enabled */

  unsigned int		current_page;	   /* register current page */
  uint8_t			tx[10];			   /* SPI translate buff */
  uint8_t			rx[4];			   /* SPI received buff */

  /* work queue data */
  struct work_s		_work;
  bool 				_running;		   /* true: measure in work queue false:not measure*/
  unsigned			_mesure_ticks;

  /* ORB data */
  orb_advert_t uorb_pub;			   /* ORB topic advertiser handle  */
};


/******************************************************************************
 * adis_write_reg_8() - Write single byte to a register
 * @adis: The adis device
 * @reg: The address of the register to be written
 * @value: The value to write
 *****************************************************************************/
static inline int adis_write_reg_8(struct adis16488_dev_s *devs, unsigned int reg,
	uint8_t val)
{
	return adis_write_reg(devs, reg, &val, 1);
}

/******************************************************************************
 * adis_write_reg_16() - Write 2 bytes to a pair of registers
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @value: Value to be written
 *****************************************************************************/
static inline int adis_write_reg_16(struct adis16488_dev_s *devs, unsigned int reg,
	uint16_t val)
{
	return adis_write_reg(devs, reg, val, 2);
}

/******************************************************************************
 * adis_write_reg_32() - write 4 bytes to four registers
 * @adis: The adis device
 * @reg: The address of the lower of the four register
 * @value: Value to be written
 *****************************************************************************/
static inline int adis_write_reg_32(struct adis16488_dev_s *devs, unsigned int reg,
	uint32_t val)
{
	return adis_write_reg(devs, reg, val, 4);
}

/******************************************************************************
 * adis_read_reg_16() - read 2 bytes from a 16-bit register
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value read back from the device
 *****************************************************************************/
static inline int adis_read_reg_16(struct adis16488_dev_s *devs, unsigned int reg,
	void *val)
{
	unsigned int tmp;
	int ret;

	ret = adis_read_reg(devs, reg, &tmp, 2);
	*((uint16_t *)val) = tmp;

	return ret;
}

/*****************************************************************************
 * adis_read_reg_32() - read 4 bytes from a 32-bit register
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value read back from the device
 *****************************************************************************/
static inline int adis_read_reg_32(struct adis16488_dev_s *devs, unsigned int reg,
	void *val)
{
	unsigned int tmp;
	int ret;

	ret = adis_read_reg(devs, reg, &tmp, 4);
	*((uint32_t *)val) = tmp;

	return ret;
}

/****************************************************************************
 * Name: adis16488_spi_config_initialize
 ****************************************************************************/
struct adis16488_config_s * adis16488_spi_config_initialize(int dev);











































/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the ADIS16488
 * driver. This structure provides information about the configuration
 * of the sensor and provides some board-specific hooks.
 *
 * This sensor driver presents two interfaces, the POSIX character driver
 * interface (fops) that is intended for use from a user application, and
 * a set of direct call entry points that are intended to be used by
 * a sensor cluster driver that is running as a kernel task (a driver to
 * driver interface).  Application tasks should not attempt to call sensor
 * cluster driver entry points.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct adis16488_reg_pair_s  /* Utility struct for the below... */
{
  uint8_t addr;            /* SPI register address */
  uint8_t value;           /* Value to be stored in the above reg on open() */
};

struct adis16488_dvr_entry_vector_s
{
  struct sensor_cluster_operations_s c;

  /* Extend the sensor cluster driver interface with a SPI DMA exchange transfer.
   * The standard driver_read and driver_write perform PIO transfers.
   * The will loop waiting on the SPI hardware and are only appropriate for
   * short data transfers.
   * Note that the first byte in the tx buffer must be a command/address
   * byte. The exchange function does not provide one. Also note that
   * the first byte stored in the rxbuffer is a garbage byte, which
   * is natural for a SPI exchange transfer. Plan your buffer accordingly.
   */

  CODE void (*driver_spiexc)(FAR void *instance_handle,
                             FAR const void *txbuffer,
                             FAR void *rxbuffer, size_t nwords);
};

struct adis16488_config_s
{
  /* Since multiple ADIS16488 can be connected to the same SPI bus we need
   * to use multiple SPI device ids which are employed by NuttX to select/
   * deselect the desired ADIS16488 chip via their chip select inputs.
   */

  int spi_devid;

  /* Initial control register configuration values. */

  uint16_t initial_cr_values_size;  /* size of the below array.
                                     * 0 = use default values. */

  /* The initial value store operations will occur in the order they
   * appear in the array.
   */

  struct adis16488_reg_pair_s *initial_cr_values;


  /* Pointer to the leaf driver's sensor_cluster_operations_s structure */

  FAR struct adis16488_dvr_entry_vector_s *sc_ops;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: adis16488_spi_config_initialize
 *
 * Description:
 *   return the adis16488 config_s
 *
 * Input Parameters:
 *   dev - number of adis16488_dev_s.
 *
 * Returned Value:
 *   config_s pointer
 *
 ****************************************************************************/
struct adis16488_config_s * adis16488_spi_config_initialize(int dev);

/****************************************************************************
 * Name: adis16488_register
 *
 * Description:
 *   Register the ADIS16488 character device as 'devpath'
 *
 * Input Parameters:
 *   spi     - An instance of the SPI interface to use to communicate with
 *             ADIS16488
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int adis16488_register(FAR struct spi_dev_s *spi,FAR int num);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_ADIS16488 && CONFIG_SPI_EXCHANGE */
#endif /* __INCLUDE_NUTTX_SENSORS_ADIS16488_H */
