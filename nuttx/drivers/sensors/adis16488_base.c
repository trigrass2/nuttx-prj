/*
 * Common library for ADIS16XXX devices
 *
 * Copyright 2012 Analog Devices Inc.
 *   Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2 or later.
 */


#include <errno.h>
#include <debug.h>
#include <string.h>
#include <semaphore.h>

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#if defined(CONFIG_SENSORS_ADIS16488)
#include <nuttx/sensors/adis16488.h>

#if defined(CONFIG_SPI)
#include <nuttx/sensors/adis16488_spi.h>
#endif /* CONFIG_SPI */


#include <nuttx/timers/drv_hrt.h>

static int      adis16488_dvr_open(FAR void *instance_handle, int32_t arg);
static int      adis16488_dvr_close(FAR void *instance_handle, int32_t arg);
static ssize_t  adis16488_dvr_read(FAR void *instance_handle,
                                 FAR char *buffer, size_t buflen);
static ssize_t  adis16488_dvr_write(FAR void *instance_handle,
                                  FAR const char *buffer, size_t buflen);
static off_t    adis16488_dvr_seek(FAR void *instance_handle, off_t offset,
                                 int whence);
static int      adis16488_dvr_ioctl(FAR void *instance_handle, int cmd,
                                  unsigned long arg);
static void     adis16488_dvr_exchange(FAR void *instance_handle,
                                     FAR const void *txbuffer,
                                     FAR void *rxbuffer, size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct adis16488_dvr_entry_vector_s g_adis16488_dops =
{
  /* Standard sensor cluster driver entry-vector */

  {
    .driver_open    = adis16488_dvr_open,
    .driver_close   = adis16488_dvr_close,
    .driver_read    = adis16488_dvr_read,
    .driver_write   = adis16488_dvr_write,
    .driver_seek    = adis16488_dvr_seek,
    .driver_ioctl   = adis16488_dvr_ioctl,
    .driver_suspend = 0,
    .driver_resume  = 0,
    },

  /* adis16488 extensions follow */

  .driver_spiexc = adis16488_dvr_exchange,
};


/****************************************************************************
 * Private data storage
 ****************************************************************************/
void 				adis16488_cycle_trampoline(void *arg);
static inline void 	adis16488_cycle(void *arg);

/* Default accelerometer initialization sequence */

/* Configure ADIS16488 to read live data (not using FIFO).
 * 1. Set to standby mode. The below can't be set while running.
 * 2. Configure the FIFO to be bypassed.
 * 3. Configure interrupts as disabled, because ADIS16488 irpts are used.
 * 4. Configure the Output Data Rate (ODR) as 1600 Hz.
 * 5. Configure normal mode (vs low noise) and 800Hz bandwidth.
 * 6. Set to operational mode; 370ms filter settle; LPF=enb; HPF=dis;
 */

static struct adis16488_reg_pair_s g_initial_adis16488_cr_values[] =
{
//  /* Set to standby mode */
//
//  {
//    .addr  = ADIS16488_POWER_CTL,
//    .value = 0
//  },
//  {
//    .addr  = ADIS16488_FIFO_CTL,
//    .value = ADIS16488_FIFO_BYPASSED
//  },
//
//  /* Interrupts disabled. */
//
//  {
//    .addr  = ADIS16488_INT1_MAP,
//    .value = 0
//  },
//  {
//    .addr  = ADIS16488_TIMING,
//    .value = ADIS16488_TIMING_ODR1600
//  },
//  {
//    .addr  = ADIS16488_MEASURE,
//    .value = ADIS16488_MEAS_BW800
//  },
//  {
//    .addr  = ADIS16488_POWER_CTL,
//    .value = ADIS16488_POWER_HPF_DISABLE | ADIS16488_POWER_MODE_MEASURE
//  }
};

static struct adis16488_config_s g_adsi16488_1_config =
{
	.spi_devid 				= SPIDEV_USER(0),
	.initial_cr_values_size = 0,
	.initial_cr_values  	= g_initial_adis16488_cr_values,
	.sc_ops 				= &g_adis16488_dops,
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void convert_acc_data(void *dst,void *src)
{
	uint16_t tmp = 0;
	float data_out = 0;

	memcpy(&tmp, src+2, 2);

	if(tmp >= 0xA81C)
	{
		data_out = (0x10000 - tmp) * (-0.8);
	}
	else
	{
		data_out = tmp * 0.8;
	}

	*(float *)dst = data_out;

}

static inline void convert_gyro_data(void *dst,void *src)
{
	uint16_t tmp = 0;
	float data_out = 0;

	memcpy(&tmp, src+2, 2);

	if(tmp >= 0xA81C){
		data_out = (0x10000 - tmp) * (-0.02);

	}else
	{
		data_out = tmp * 0.02;
	}

	*(float *)dst = data_out;

}

static inline void convert_mag_data(void *dst,void *src)
{
	uint16_t tmp = 0;
	float data = 0;

	memcpy(&tmp, src, 2);

	if(tmp >= 0xA81C){
		data = (0x10000 - tmp) * (-0.1);

	}else{
		data = tmp *0.1;
	}
	*(float *)dst = data;

}

static inline void convert_delta_v_data(void *dst,void *src)
{
	uint16_t tmp = 0;
	float data = 0;

	memcpy(&tmp, src, 2);

	if(tmp >= 0xA81C){
		data = (0x10000 - tmp) * (-200.0/32768.0);

	}else{
		data = tmp *200.0/32768.0;
	}
	*(float *)dst = data;

}

static inline void convert_angle_data(void *dst,void *src)
{
	uint16_t tmp = 0;
	float data = 0;

	memcpy(&tmp, src, 2);

	if(tmp >= 0xA81C){
		data = (0x10000 - tmp) * (-720.0/32768.0);

	}else{
		data = tmp *720.0/32768;
	}
	*(float *)dst = data;

}

/****************************************************************************
 * Name: adis16488_measure
 *
 * Description:
 *   measure sensor from adis16488.
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
struct adis16488_uorb_s   adis16488_data;
static  inline void adis16488_measure(void *arg)
{
	FAR struct adis16488_dev_s *dev =  arg;

	uint16_t ret = 0;

	/***********************************************************************
	 * measure the sensor
	 ***********************************************************************/
	//accelerometer
	uint8_t x_acc[4],y_acc[4],z_acc[4];
	ret = adis_read_reg_32(dev, ADIS16488_REG_X_ACCEL_OUT, &x_acc);
	if(ret < 0){
		snerr("ERROR: Failed to read accelerometer:%d\n",ret);
	}

	ret = adis_read_reg_32(dev, ADIS16488_REG_Y_ACCEL_OUT, &y_acc);
	if(ret < 0){
		snerr("ERROR: Failed to read accelerometer:%d\n",ret);
	}

	ret = adis_read_reg_32(dev, ADIS16488_REG_Z_ACCEL_OUT, &z_acc);
	if(ret < 0){
		snerr("ERROR: Failed to read accelerometer:%d\n",ret);
	}

	//gyroscale
	uint8_t x_gyro[4],y_gyro[4],z_gyro[4];
	ret = adis_read_reg_32(dev, ADIS16488_REG_X_GYRO_OUT, &x_gyro);
	if(ret < 0){
		snerr("ERROR: Failed to read gyroscale:%d\n",ret);
	}

	ret = adis_read_reg_32(dev, ADIS16488_REG_Y_GYRO_OUT, &y_gyro);
	if(ret < 0){
		snerr("ERROR: Failed to read gyroscale:%d\n",ret);
	}

	ret = adis_read_reg_32(dev, ADIS16488_REG_Z_GYRO_OUT, &z_gyro);
	if(ret < 0){
		snerr("ERROR: Failed to read gyroscale:%d\n",ret);
	}

	//magnetometer
	uint8_t x_mag[2],y_mag[2],z_mag[2];
	ret = adis_read_reg_16(dev, ADIS16488_REG_X_MAGN_OUT, &x_mag);
	if(ret < 0){
		snerr("ERROR: Failed to read magnetometer:%d\n",ret);
	}

	ret = adis_read_reg_16(dev, ADIS16488_REG_Y_MAGN_OUT, &y_mag);
	if(ret < 0){
		snerr("ERROR: Failed to read magnetometer:%d\n",ret);
	}

	ret = adis_read_reg_16(dev, ADIS16488_REG_Z_MAGN_OUT, &z_mag);
	if(ret < 0){
		snerr("ERROR: Failed to read magnetometer:%d\n",ret);
	}

	//delta_vel
	uint8_t x_vel[4], y_vel[4],z_vel[4];
	ret = adis_read_reg_32(dev, ADIS16488_REG_X_DELTAVEL_OUT, &x_vel);
	if(ret < 0){
		snerr("ERROR: Failed to read delta_vel:%d\n",ret);
	}

	ret = adis_read_reg_32(dev, ADIS16488_REG_Y_DELTAVEL_OUT, &y_vel);
	if(ret < 0){
		snerr("ERROR: Failed to read delta_vel:%d\n",ret);
	}

	ret = adis_read_reg_32(dev, ADIS16488_REG_Z_DELTAVEL_OUT, &z_vel);
	if(ret < 0){
		snerr("ERROR: Failed to read delta_vel:%d\n",ret);
	}

	//angle
	uint8_t x_ang[4], y_ang[4], z_ang[4];
	ret = adis_read_reg_32(dev, ADIS16488_REG_X_DELTAANG_OUT, &x_ang);
	if(ret < 0){
		snerr("ERROR: Failed to read angle:%d\n",ret);
	}

	ret = adis_read_reg_32(dev, ADIS16488_REG_Y_DELTAANG_OUT, &y_ang);
	if(ret < 0){
		snerr("ERROR: Failed to read angle:%d\n",ret);
	}

	ret = adis_read_reg_32(dev, ADIS16488_REG_Z_DELTAANG_OUT, &y_ang);
	if(ret < 0){
		snerr("ERROR: Failed to read angle:%d\n",ret);
	}


	/***********************************************************************
	 * convert the data
	 ***********************************************************************/

	//acc
	convert_acc_data(&(dev->sdata.acc[X_AXIS]),&x_acc[0]);
	convert_acc_data(&(dev->sdata.acc[Y_AXIS]),&y_acc[0]);
	convert_acc_data(&(dev->sdata.acc[Z_AXIS]),&z_acc[0]);

	//gyro
	convert_gyro_data(&(dev->sdata.gyro[X_AXIS]),&x_gyro[0]);
	convert_gyro_data(&(dev->sdata.gyro[Y_AXIS]),&y_gyro[0]);
	convert_gyro_data(&(dev->sdata.gyro[Z_AXIS]),&z_gyro[0]);

	//mag
	convert_mag_data(&(dev->sdata.mag[X_AXIS]),&x_mag[0]);
	convert_mag_data(&(dev->sdata.mag[Y_AXIS]),&y_mag[0]);
	convert_mag_data(&(dev->sdata.mag[Z_AXIS]),&z_mag[0]);

	//delta_vel
	convert_delta_v_data(&(dev->sdata.delta_v[X_AXIS]),&x_vel[2]);
	convert_delta_v_data(&(dev->sdata.delta_v[Y_AXIS]),&y_vel[2]);
	convert_delta_v_data(&(dev->sdata.delta_v[Z_AXIS]),&z_vel[2]);

	//angle
	convert_angle_data(&(dev->sdata.angle[X_AXIS]),&x_ang[2]);
	convert_angle_data(&(dev->sdata.angle[Y_AXIS]),&y_ang[2]);
	convert_angle_data(&(dev->sdata.angle[Z_AXIS]),&z_ang[2]);

	//copy measure data to orb data

	adis16488_data.acc[X_AXIS] = -0.001 * dev->sdata.acc[X_AXIS];
	adis16488_data.acc[Y_AXIS] =  0.001 * dev->sdata.acc[Y_AXIS];
	adis16488_data.acc[Z_AXIS] =  0.001 * dev->sdata.acc[Z_AXIS];

	adis16488_data.gyro[X_AXIS] =  DEG2RAD * dev->sdata.gyro[X_AXIS];
	adis16488_data.gyro[Y_AXIS] = -DEG2RAD * dev->sdata.gyro[Y_AXIS];
	adis16488_data.gyro[Z_AXIS] = -DEG2RAD * dev->sdata.gyro[Z_AXIS];

	adis16488_data.mag[X_AXIS] =  dev->sdata.mag[X_AXIS];
	adis16488_data.mag[Y_AXIS] = -dev->sdata.mag[Y_AXIS];
	adis16488_data.mag[Z_AXIS] = -dev->sdata.mag[Z_AXIS];

	memcpy(adis16488_data.delta_v,dev->sdata.delta_v,sizeof(dev->sdata.delta_v));
	memcpy(adis16488_data.angle  ,dev->sdata.angle  ,sizeof(dev->sdata.angle));

	//publish orb message
	orb_publish(ORB_ID(adis16488_uorb), dev->uorb_pub, &adis16488_data);
}


/****************************************************************************
 * Name: adis16488_cycle
 *
 * Description:
 *    cycle call work_queue
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/

static inline void adis16488_cycle(void *arg)
{
	FAR struct adis16488_dev_s *dev =  arg;

	/* measure sensor via work queue */
	adis16488_measure(arg);

	if (dev->_running) {
		/* schedule a cycle to start things */
		work_queue(HPWORK, &dev->_work, (worker_t)adis16488_cycle_trampoline, dev, dev->_mesure_ticks);
	}
}

/****************************************************************************
 * Name: adis16488_cycle_trampoline
 *
 * Description:
 *   subscribe the uorb message.
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
void adis16488_cycle_trampoline(void *arg)
{
	FAR struct adis16488_dev_s *dev =  arg;

	adis16488_cycle(dev);
}

/****************************************************************************
 * Name: adis16488_start_cycle
 *
 * Description:
 *    start cycle call work_queue
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
static void adis16488_start_cycle(void *arg)
{
	FAR struct adis16488_dev_s *dev =  arg;

	if(!dev->_running){

		/* reset the data and state */
		memset(&adis16488_data, 0, sizeof(adis16488_data));

		/* advertise the message topic */
		dev->uorb_pub = orb_advertise(ORB_ID(adis16488_uorb), &adis16488_data);


		/* schedule a cycle to start things */
		dev->_running = true;
		work_queue(HPWORK, &dev->_work, (worker_t)adis16488_cycle_trampoline, dev, 1);
	}

}


/****************************************************************************
 * Name: adis16488_stop_cycle
 *
 * Description:
 *    stop cycle call work_queue
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
static void
adis16488_stop_cycle(void *arg)
{
	FAR struct adis16488_dev_s *dev =  arg;

	int ret = 0;

	if(dev->_running){

		/* stop the cycle */
		dev->_running = false;

		work_cancel(HPWORK, &dev->_work);

		/* unadvertise the message topic */

		ret = orb_unadvertise(dev->uorb_pub);
		if(ret <  0){
			syslog(LOG_ERR,"orb_unadvertise: failed to unadvertise uorb topic:%d",ret);
		}
	}
}



/****************************************************************************
 * adis16488_reset() - Reset the device
 * @adis: The adis device
 *
 * Returns 0 on success, a negative error code otherwise
 ****************************************************************************/
int adis16488_reset(FAR struct adis16488_dev_s *dev)
{
	int ret;

	ret = adis_write_reg_8(dev, ADIS16488_REG_GLOB_CMD, ADIS_GLOB_CMD_SOFTWARE_RESET);
	if (ret){
		snerr("ERROR: Failed to reset device: %d\n", ret);
	}
	usleep(500000);
	return ret;
}

/****************************************************************************
 * Name: adis16488_read_id
 *
 * Description:
 *
 *   Read the ADIS16488's ID Registers.
 *   There are one ID Register...
 *
 *     Manufacturer should be ADIS16488_DEVID_AD_VALUE (0x4068)= 16488.
 *
 ****************************************************************************/

static uint16_t adis16488_read_id(FAR struct adis16488_dev_s *dev)
{
	uint16_t devid = 0;
	uint16_t ret = 0;

	ret = adis_read_reg_16(dev, ADIS16488_REG_PROD_ID, &devid);
	if(ret < 0){
		snerr("ERROR: Failed read device id.\n");
		devid = ret;
	}
	return devid;
}

/****************************************************************************
 * Name: adis16488_read_firmware_revision
 *
 * Description:
 *
 *   Read the ADIS16488's firmware revision Registers.
 *
 ****************************************************************************/

static uint16_t adis16488_read_firmware_revision(FAR struct adis16488_dev_s *dev)
{
	uint16_t recv;
	int ret;

	ret = adis_read_reg_16(dev, ADIS16488_REG_FIRM_REV, &recv);
	if (ret < 0){
		snerr("ERROR: Failed read device firmware revision.\n");
		return ret;
	}

	return recv;
}


/****************************************************************************
 * Name: adis16488_read_serial_number
 *
 * Description:
 *
 *   Read the ADIS16488's firmware revision Registers.
 *
 ****************************************************************************/

static int32_t adis16488_read_serial_number(FAR struct adis16488_dev_s *dev)
{
	uint16_t serial;
	int ret;

	ret = adis_read_reg_16(dev, ADIS16488_REG_SERIAL_NUM,&serial);
	if (ret < 0){
		snerr("ERROR: Failed read device serial_number.\n");
		return ret;
	}

	return (int32_t)serial;
}

/*****************************************************************************
 * adis16488_check_status() - Check the device for error conditions
 * @adis: The adis device
 *
 * Returns 0 on success, a negative error code otherwise
 *****************************************************************************/
int adis16488_check_status(FAR struct adis16488_dev_s *dev,uint32_t cmd_reg)
{
	uint16_t status;
	int ret;

	ret = adis_read_reg_16(dev, cmd_reg, &status);
	if (ret < 0){
		snerr("ERROR: [reg:%04x] Failed to check device status: %d\n", cmd_reg, ret);
		return ret;
	}

	status &= dev->data.status_error_mask;

	if (status == 0){
		return OK;
	}else{
		return status;
	}

	return -EIO;
}


/****************************************************************************
 * adis16488_reset() - Reset the device
 * @adis: The adis device
 *
 * Returns 0 on success, a negative error code otherwise
 ****************************************************************************/
int adis16488_self_test(FAR struct adis16488_dev_s *dev)
{
	int ret;

	ret = adis_write_reg_8(dev, ADIS16488_REG_GLOB_CMD,  ADIS_GLOB_CMD_SELF_TEST);
	if (ret < 0){
		snerr("ERROR: Failed to reset device: %d\n", ret);
		return ret;
	}

	usleep(15*1000);
//	usleep(dev->data->startup_delay);

	ret = adis16488_check_status(dev,ADIS16488_REG_DIAG_STS);
	if (ret == 0){
		sninfo("INFO:  Check system status status OK\n" ,ret);
		return OK;
	}else{
		snwarn("WARN: System status: %04x\n" ,ret);
		return ret;
	}

	return ret;
}


/****************************************************************************
 * Name: adis16488_dvr_open
 ****************************************************************************/

static int adis16488_dvr_open(FAR void *instance_handle, int32_t arg)
{
  FAR struct adis16488_dev_s *priv = (FAR struct adis16488_dev_s *)instance_handle;
//  FAR struct adis16488_reg_pair_s *initp;
  uint16_t pnpid = 0 ,vm = 0;
  uint32_t sn = 0;

  sninfo("adis16488_open: entered...\n");

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  /* Read the ID registers */

  pnpid = adis16488_read_id(priv);
  priv->readonly = false;

  sninfo("INFO: ADIS16488 ID = 0x%04x\n", pnpid);

  if ((pnpid !=16488))
    {
      snwarn("WARN: Invalid ADIS16488 ID = 0x%04x\n", pnpid);

      priv->readonly = true;
    }
  else /* ID matches */
    {
	  /* Perform a sensor reset */
	  adis16488_reset(priv);

	  /* Perform a sensor self-detect */
	  adis16488_self_test(priv);

	  /* Read sensor firmware vision */
	  vm =  adis16488_read_firmware_revision(priv);
	  if (vm > 0){
		  sninfo("INFO: Firmware Revision = 0x%04x\n", vm);
	  }
	  
	  /* Read sensor firmware serial number */
	  sn = adis16488_read_serial_number(priv);
	  if (sn > 0){
		  sninfo("INFO: serial number = 0x%08x\n", sn);
	  }

      /* Choose the initialization sequence */

	  //todo:need to implemented

      /* Apply the initialization sequence */

	  //todo:need to implemented
    }
  return OK;
}

/****************************************************************************
 * Name: adis16488_dvr_close
 ****************************************************************************/

static int adis16488_dvr_close(FAR void *instance_handle, int32_t arg)
{
  FAR struct adis16488_dev_s *priv = (FAR struct adis16488_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);
  UNUSED(arg);

  /* Perform a reset to place the sensor in standby mode.*/

  adis16488_reset(priv);

  return OK;
}

/****************************************************************************
 * Name: adis16488_dvr_read
 ****************************************************************************/

static ssize_t adis16488_dvr_read(FAR void *instance_handle, FAR char *buffer,
                                size_t buflen)
{
  FAR struct adis16488_dev_s *priv = ((FAR struct adis16488_dev_s *)instance_handle);

  int ret;

  DEBUGASSERT(priv != NULL);

  /* Permute data out fields */
  ret = adis_read_reg(priv, priv->seek_address, (uint32_t *)buffer, buflen);
  if (ret < 0){
	  return ret;
  }

  return buflen;
}

/****************************************************************************
 * Name: adis16488_dvr_write
 ****************************************************************************/

static ssize_t adis16488_dvr_write(FAR void *instance_handle,
                                 FAR const char *buffer, size_t buflen)
{
  FAR struct adis16488_dev_s *priv = (FAR struct adis16488_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  int ret;
  if (priv->readonly){
      return -EROFS;
    }

  ret = adis_write_reg(priv, priv->seek_address, (uint32_t *)buffer, buflen);
  if (ret < 0){
	  return ret;
  }

  return buflen;
}

/****************************************************************************
 * Name: adis16488_dvr_seek
 ****************************************************************************/

static off_t adis16488_dvr_seek(FAR void *instance_handle, off_t offset,
                              int whence)
{
  FAR struct adis16488_dev_s *priv = (FAR struct adis16488_dev_s *)instance_handle;

  DEBUGASSERT(priv != NULL);

  uint16_t reg;
  switch (whence)
    {
      case SEEK_CUR:  /* Incremental seek */
        reg = priv->seek_address + offset;
        if (0 > reg || reg > ADIS16488_FIR_COEF_D(119)){
            return -EINVAL;
          }

        priv->seek_address = reg;
        break;

      case SEEK_END:  /* Seek to the 1st X-data register */
        priv->seek_address = ADIS16488_FIR_COEF_D(119);
        break;

      case SEEK_SET:  /* Seek to designated address */
        if (0 > offset || offset > ADIS16488_FIR_COEF_D(119))
          {

            return -EINVAL;
          }

        priv->seek_address = offset;
        break;

      default:        /* invalid whence */

        return -EINVAL;
    }

  return priv->seek_address;
}

/****************************************************************************
 * Name: adis16488_dvr_ioctl
 ****************************************************************************/

static int adis16488_dvr_ioctl(FAR void *instance_handle, int cmd,
                             unsigned long arg)
{
  FAR struct adis16488_dev_s *priv = (FAR struct adis16488_dev_s *)instance_handle;
  int ret = OK;

  switch (cmd)
    {
      /* Command was not recognized */

		case ADISIOC_MEASUREMENT_RATE_SET:
			priv->_mesure_ticks = USEC2TICK(arg);
		break;

		case ADISIOC_MEASUREMENT_START:
			adis16488_start_cycle(priv);
		break;

		case ADISIOC_MEASUREMENT_STOP:
			adis16488_stop_cycle(priv);
		break;

		default:
		  snerr("ERROR: Unrecognized cmd: %d\n", cmd);
		  ret = -ENOTTY;
		  break;
    }

  return ret;
}

/****************************************************************************
 * Name: adis16488_dvr_exchange (with SPI DMA capability)
 *
 * Description:
 *   Exchange a block of data on SPI using DMA
 *
 * Input Parameters:
 *   instance_handle - Pointer to struct adis16488_dev_s.
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adis16488_dvr_exchange(FAR void *instance_handle,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer, size_t nwords)
{
  FAR struct adis16488_dev_s *priv = (FAR struct adis16488_dev_s *)instance_handle;
  FAR struct spi_dev_s *spi = priv->spi;

  /* Lock the SPI bus so that only one device can access it at the same time */

  SPI_LOCK(spi, true);

  SPI_SETFREQUENCY(spi, ADIS16488_SPI_FREQUENCY);

  SPI_SETMODE(spi, ADIS16488_SPI_MODE);

  /* Set CS to low which selects the ADIS16488 */

  SPI_SELECT(spi, priv->config->spi_devid, true);

  /* Perform an SPI exchange block operation. */

  SPI_EXCHANGE(spi, txbuffer, rxbuffer, nwords);

  /* Set CS to high to deselect the ADIS16488 */

  SPI_SELECT(spi, priv->config->spi_devid, false);

  /* Unlock the SPI bus */

  SPI_LOCK(spi, false);
 }



/****************************************************************************
 * Name: adis16488_spi_config_initialize
 * Description:
 *   return the adis16488 config_s
 * Input Parameters:
 *   dev - number of adis16488_dev_s.
 * Returned Value:
 *   config_s pointer
 ****************************************************************************/
struct adis16488_config_s * adis16488_spi_config_initialize(int dev)
{
	switch(dev)
	{
		case 1:
		{
			return &g_adsi16488_1_config;
		}
		break;

		default:
			return NULL;
			break;
	}
	return NULL;
}


#endif /* CONFIG_SENSORS_ADIS16488 */
