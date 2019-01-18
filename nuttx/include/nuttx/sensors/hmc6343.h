
/******************************************************************************
 * include/nuttx/sensors/hmc6343.h
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
/**************************************************************************/
/*
 *  Distributed with a free-will license.
 *  Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
 *  HMC6343
 *  This code is designed to work with the HMC6343_I2CS I2C Mini Module available from ControlEverything.com.
 *  https://www.controleverything.com/content/Accelorometer?sku=HMC6343_I2CS#tabs-0-product_tabset-2
*/
/**************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/cluster_driver.h>

#include "../../../../apps/include/uORB/uorb/uORB.h"
#include "../../../../apps/include/uORB/topic/hmc6343_uorb.h"

/**************************************************************************
I2C ADDRESS/BITS
**************************************************************************/
#define HMC6343_I2C_DEFAULT_ADDRESS                (0x19)	   // 7bits address (0x32 = 0x19<<1 + 0:read) (write:0x33 = 0x19<<1 + 1:write)

#define HMC6343_I2C_STANDARD_FREQUENCY             (100000)	   //
#define HMC6343_I2C_FAST_FREQUENCY                 (200000)	   //
/**************************************************************************
CONVERSION DELAY (in mS)x
**************************************************************************/
#define HMC6343_CONVERSIONDELAY                    (100)

/**************************************************************************
DEVICE STATUS
**************************************************************************/
#define HMC6343_STAT_INITIALIZED  				   (0x01) /* Device has been initialized */

/**************************************************************************
REGISTERS DESCRIPTION
**************************************************************************/
#define HMC6343_REG_I2C_SLAVE_ADDRESS		       (0x00)      // I2C Slave Address Register
#define HMC6343_REG_ACCELACCELMAG_SW_VERSION       (0x02)      // Software Version Number Register
#define HMC6343_OMR1              				   (0x04)      // Operational Mode Register 1
#define HMC6343_OMR2              				   (0x05)      // Operational Mode Register 2
#define HMC6343_REG_ACCELMAG_SN_LSB                (0x06)      // Device Serial Number Register
#define HMC6343_REG_ACCELMAG_SN_MSB                (0x07)      // Device Serial Number Register
#define HMC6343_REG_ACCELMAG_DEVIATION_LSB         (0x0A)      // Deviation Angle (±1800) in Tenths of a Degree Register
#define HMC6343_REG_ACCELMAG_DEVIATION_MSB         (0x0B)      // Deviation Angle (±1800) in Tenths of a Degree Register
#define HMC6343_REG_ACCELMAG_VARIATION_LSB         (0x0C)      // Variation Angle (±1800) in Tenths of a Degree Register
#define HMC6343_REG_ACCELMAG_VARIATION_MSB         (0x0D)      // Variation Angle (±1800) in Tenths of a Degree Register
#define HMC6343_REG_ACCELMAG_XOFFSET_LSB           (0x0E)      // Hard-Iron Calibration Offset for the X-Axis Register
#define HMC6343_REG_ACCELMAG_XOFFSET_MSB           (0x0F)      // Hard-Iron Calibration Offset for the X-Axis Register
#define HMC6343_REG_ACCELMAG_YOFFSET_LSB           (0x10)      // Hard-Iron Calibration Offset for the Y-Axis Register
#define HMC6343_REG_ACCELMAG_YOFFSET_MSB           (0x11)      // Hard-Iron Calibration Offset for the Y-Axis Register
#define HMC6343_REG_ACCELMAG_ZOFFSET_LSB           (0x12)      // Hard-Iron Calibration Offset for the Z-Axis Register
#define HMC6343_REG_ACCELMAG_ZOFFSET_MSB           (0x13)      // Hard-Iron Calibration Offset for the Z-Axis Register
#define HMC6343_REG_ACCELMAG_FILTER_LSB            (0x14)      // Heading IIR Filter (0x00 to 0x0F typical) Register
#define HMC6343_REG_ACCELMAG_FILTER_MSB            (0x15)      // Heading IIR Filter (set at zero) Register
#define HMC6343_REG_ACCELMAG_POST_ACCEL            (0x40)      // 6 Binary Data Bytes. AxMSB, AxLSB, AyMSB, AyLSB, AzMSB, AzLSB
#define HMC6343_REG_ACCELMAG_POST_MAG              (0x45)      // 6 Binary Data Bytes. MxMSB, MxLSB, MyMSB, MyLSB, MzMSB, MzLSB
#define HMC6343_REG_ACCELMAG_POST_HEADING          (0x50)      // 6 Binary Data Bytes. HeadMSB, HeadLSB, PitchMSB, PitchLSB, RollMSB, RollLSB
#define HMC6343_REG_ACCELMAG_POST_TILT             (0x55)      // 6 Binary Data Bytes. PitchMSB, PitchLSB, RollMSB, RollLSB, TempMSB, TempLSB
#define HMC6343_REG_ACCELMAG_POST_OP_MODE1         (0x65)      // OP Mode 1 Register
#define HMC6343_REG_ACCELMAG_USER_CALIB_MODE_ENTER (0x71)      // No Response Data
#define HMC6343_REG_ACCELMAG_LEVEL_ORIENTATION     (0x72)      // (X=forward, +Z=up) (default) No Response Data
#define HMC6343_REG_ACCELMAG_UPRIGHT_SIDE_ORIENT   (0x73)      // (X=forward, Y=up) No Response Data
#define HMC6343_REG_ACCELMAG_UPRIGHT_FLAT_ORIENT   (0x74)      // (Z=forward, -X=up) No Response Data
#define HMC6343_REG_ACCELMAG_RUM_MODE              (0x75)      // No Response Data
#define HMC6343_REG_ACCELMAG_STANDBY_MODE          (0x76)      // No Response Data
#define HMC6343_REG_ACCELMAG_USER_CALIB_MODE_EXIT  (0x7E)      // Identification Register A
#define HMC6343_REG_ACCELMAG_RESET_PROCESSOR       (0x82)      // No Response Data
#define HMC6343_REG_ACCELMAG_SLEEP_MODE_ENTER      (0x83)      // No Response Data
#define HMC6343_REG_ACCELMAG_SLEEP_MODE_EXIT       (0x84)      // No Response Data
#define HMC6343_REG_ACCELMAG_READ_EEPROM           (0xE1)      // 1 Binary Data Byte
#define HMC6343_REG_ACCELMAG_WRITE_EEPROM          (0xF1)      // No Response Data. Data Settling Time

/*   Operational Mode Register 1 (EEPROM 0x04) */
#define HMC6343_OMR1_LEVEL           	(1 << 0)  /* Bit 0: Level Orientation if set. */
#define HMC6343_OMR1_UE              	(1 << 1)  /* Bit 1: Upright Edge Orientation if set. */
#define HMC6343_OMR1_UF              	(1 << 2)  /* Bit 2: Upright Front Orientation if set. */
#define HMC6343_OMR1_STDBY           	(1 << 3)  /* Bit 3: Standby Mode if set. */
#define HMC6343_OMR1_RUN           		(1 << 4)  /* Bit 4: Run Mode if set. */
#define HMC6343_OMR1_IIR_FILTER        	(1 << 5)  /* Bit 5: IIR Heading Filter used if set. */
#define HMC6343_OMR1_CAL_OFS           	(1 << 6)  /* Bit 6: Calculating calibration offsets if set. (read only). */
#define HMC6343_OMR1_CAL_COMPASS       	(1 << 7)  /* Bit 7: Calculating compass data if set. (read only). */

/*   Operational Mode Register 2 (EEPROM 0x05) */
#define HMC6343_OMR2_MR_1HZ           	(1 << 0)  /* Bit [0,1]: 0:1HZ. */
#define HMC6343_OMR2_MR_5HZ             (1 << 1)  /* Bit [0,1]: 1:5HZ. */
#define HMC6343_OMR2_MR_10HZ            (1 << 2)  /* Bit [0,1]: 2:5HZ. */

/*   hmc6343 ioctl definitions */
#define HMC6343IOC_UPDATING_RATE_SET			_SNIOC(1)
#define HMC6343IOC_SOFTWARE_VERSION_GET			_SNIOC(2)
#define HMC6343IOC_SERIAL_NUMBER_GET			_SNIOC(3)
#define HMC6343IOC_ACCELEROMETER_GET			_SNIOC(4)
#define HMC6343IOC_MAGNETOMETER_GET				_SNIOC(5)
#define HMC6343IOC_HEADING_DATA_GET				_SNIOC(6)
#define HMC6343IOC_TILT_DATA_GET				_SNIOC(7)
#define HMC6343IOC_MEASUREMENT_RATE_SET			_SNIOC(8)
#define HMC6343IOC_MEASUREMENT_START			_SNIOC(9)
#define HMC6343IOC_MEASUREMENT_STOP				_SNIOC(10)

struct hmc6343_sample_s
{
    int16_t X;
    int16_t Y;
    int16_t Z;
};

enum
{
	OMR1 = 0,
	OMR2,
	REG_MAX
};

struct hmc6343_reg_pair_s  /* Utility struct for the below... */
{
  uint8_t addr;            /* SPI register address */
  uint8_t value;           /* Value to be stored in the above reg on open() */
};

/* A reference to a structure of this type must be passed to the HMC6343 driver when the
 * driver is instantiated. This structure provides information about the configuration of the
 * HMC6343 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied by the driver
 * and is presumed to persist while the driver is active. The memory must be writeable
 * because, under certain circumstances, the driver may modify the frequency.
 */

struct hmc6343_config_s
{
  /* Device characterization */

  uint8_t address;     							/* 7-bit I2C address (only bits 0-6 used) */

  uint32_t frequency;  							/* I2C frequency */

  struct hmc6343_reg_pair_s *initial_cr_values; /* Utility struct for the below... */

  struct sensor_cluster_operations_s *sc_ops; 	/* Cluster driver operations interface */
};


/* This structure represents the state of the HMC6343 driver */

struct hmc6343_dev_s
{
  /* Common fields */

  FAR struct hmc6343_config_s *config; /* Board configuration data */
  FAR struct i2c_master_s *i2c;        /* Saved I2C driver instance */

  /* work queue fields */
  struct work_s _work;                  /* Supports the interrupt handling "bottom half" */
  bool 				_running;		   /* true: measure in work queue false:not measure*/
  unsigned			_mesure_ticks;     /* cycle frequency in work queue  */

  /* semaphore fields */
  sem_t exclsem;                       /* Manages exclusive access to this structure */
  sem_t waitsem;                       /* Used to wait for the availability of data */

  /* status fields */
  uint8_t status;                      /* See HMC6343_STAT_* definitions */
  uint8_t crefs;                       /* Number of times the device has been opened */
  uint8_t nwaiters;                    /* Number of threads waiting for HMC6343 data */

  uint16_t ofsx;                       /* Offset X value */
  uint16_t ofsy;                       /* Offset Y value */
  uint16_t ofsz;                       /* Offset Z value */

  struct hmc6343_sample_s sample;      /* Last sampled Compass data */

  /* ORB data */
  orb_advert_t uorb_pub;			   /* ORB topic advertiser handle  */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: hmc6343_register
 *
 * Description:
 *  This function will register the touchscreen driver as /dev/accelN where N
 *  is the minor device number
 *
 * Input Parameters:
 *   handle    - The handle previously returned by hmc6343_register
 *   minor     - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int hmc6343_register(FAR struct i2c_master_s *dev, int minor);


