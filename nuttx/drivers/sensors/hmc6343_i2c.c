/****************************************************************************
 * drivers/sensors/hmc6343_i2c.c
 * Character driver for the ST HMC6343 Three-axis Compass with Algorithms.
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

#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/sensors/hmc6343.h>

#if defined(CONFIG_I2C) \
	&& defined(CONFIG_SENSORS_HMC6343)

static struct hmc6343_uorb_s hmc6343_orb_data;


void hmc6343_cycle_trampoline(void *arg);
static inline void hmc6343_cycle(void *arg);


/****************************************************************************
 * Name: hmc6343_measure
 *
 * Description:
 *   measure sensor from hmc6343.
 *
 * Input Parameters:
 *   arg  - device pointer
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
static inline int hmc6343_measure(void *arg)
{
	FAR struct hmc6343_dev_s *priv =  arg;
	struct i2c_msg_s msg[2];
	uint8_t reg ,buff[6];
	uint16_t ret = 0;

	/***********************************************************************
	 * measure the sensor
	 ***********************************************************************/
	/*
	 *  Get device hmc6343 accelerometer
	 */

	reg = HMC6343_REG_ACCELMAG_POST_ACCEL;

	msg[0].frequency = priv->config->frequency;
	msg[0].addr      = priv->config->address;
	msg[0].flags     = 0;
	msg[0].buffer    = &reg;
	msg[0].length    = 1;

	msg[1].frequency = priv->config->frequency;
	msg[1].addr      = priv->config->address;
	msg[1].flags     = I2C_M_READ;
	msg[1].buffer    = (uint8_t *)buff;
	msg[1].length    = 6;

	ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
	if(ret < 0){
		snerr("ERROR: Failed to get hmc6343 accelerometer: %d\n", ret);
		return ret;
	}

	hmc6343_orb_data.acc[0] = (int16_t)((buff[0] << 8) | buff[1]);
	hmc6343_orb_data.acc[1] = (int16_t)((buff[2] << 8) | buff[3]);
	hmc6343_orb_data.acc[2] = (int16_t)((buff[4] << 8) | buff[5]);


	/*
	 * Get device hmc6343 magnetometer
	 */
	reg = HMC6343_REG_ACCELMAG_POST_MAG;

	msg[0].frequency = priv->config->frequency;
	msg[0].addr      = priv->config->address;
	msg[0].flags     = 0;
	msg[0].buffer    = &reg;
	msg[0].length    = 1;

	msg[1].frequency = priv->config->frequency;
	msg[1].addr      = priv->config->address;
	msg[1].flags     = I2C_M_READ;
	msg[1].buffer    = (uint8_t *)buff;
	msg[1].length    = 6;

	ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
	if(ret < 0){
		snerr("ERROR: Failed to get hmc6343 magnetometer: %d\n", ret);
		return ret;
	}

	hmc6343_orb_data.mag[0] = (int16_t)((buff[0] << 8) | buff[1]);
	hmc6343_orb_data.mag[1] = (int16_t)((buff[2] << 8) | buff[3]);
	hmc6343_orb_data.mag[2] = (int16_t)((buff[4] << 8) | buff[5]);

	/*
	 *  Get device hmc6343 heading data
	 */
	reg = HMC6343_REG_ACCELMAG_POST_HEADING;

	msg[0].frequency = priv->config->frequency;
	msg[0].addr      = priv->config->address;
	msg[0].flags     = 0;
	msg[0].buffer    = &reg;
	msg[0].length    = 1;

	msg[1].frequency = priv->config->frequency;
	msg[1].addr      = priv->config->address;
	msg[1].flags     = I2C_M_READ;
	msg[1].buffer    = (uint8_t *)buff;
	msg[1].length    = 6;

	ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
	if(ret < 0){
		snerr("ERROR: Failed to get hmc6343 heading: %d\n", ret);
		return ret;
	}

	hmc6343_orb_data.heading[0] = (int16_t)((buff[0] << 8) | buff[1]);
	hmc6343_orb_data.heading[1] = (int16_t)((buff[2] << 8) | buff[3]);
	hmc6343_orb_data.heading[2] = (int16_t)((buff[4] << 8) | buff[5]);


	/***********************************************************************
	 * publish orb message
	 ***********************************************************************/

	ret = orb_publish(ORB_ID(hmc6343_uorb), priv->uorb_pub, &hmc6343_orb_data);

	return ret;
}


/****************************************************************************
 * Name: hmc6343_cycle
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

static inline void hmc6343_cycle(void *arg)
{
	FAR struct hmc6343_dev_s *dev =  arg;

	/* measure sensor via work queue */
	hmc6343_measure(arg);

	if (dev->_running) {
		/* schedule a cycle to start things */
		work_queue(LPWORK, &dev->_work, (worker_t)hmc6343_cycle_trampoline, dev, dev->_mesure_ticks);
	}
}

/****************************************************************************
 * Name: hmc6343_cycle_trampoline
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
void hmc6343_cycle_trampoline(void *arg)
{
	FAR struct hmc6343_dev_s *dev =  arg;

	hmc6343_cycle(dev);
}

/****************************************************************************
 * Name: hmc6343_start_cycle
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

static void hmc6343_start_cycle(void *arg)
{
	FAR struct hmc6343_dev_s *dev =  arg;

	if(!dev->_running){

		/* reset the data and state */
		memset(&hmc6343_orb_data, 0, sizeof(hmc6343_orb_data));

		/* advertise the message topic */
		dev->uorb_pub = orb_advertise(ORB_ID(hmc6343_uorb), &hmc6343_orb_data);


		/* schedule a cycle to start things */
		dev->_running = true;
		work_queue(LPWORK, &dev->_work, (worker_t)hmc6343_cycle_trampoline, dev, 1);
	}
}


/****************************************************************************
 * Name: hmc6343_stop_cycle
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
static void hmc6343_stop_cycle(void *arg)
{
	FAR struct hmc6343_dev_s *dev =  arg;

	int ret = 0;

	if(dev->_running){

		/* stop the cycle */
		dev->_running = false;

		work_cancel(LPWORK, &dev->_work);

		/* unadvertise the message topic */

		ret = orb_unadvertise(dev->uorb_pub);
		if(ret <  0){
			syslog(LOG_ERR,"orb_unadvertise: failed to unadvertise uorb topic:%d",ret);
		}
	}

}


/****************************************************************************
 * Name: hmc6343_write
 *
 * Description:
 *   Send a block of data on I2C. Each write operation will be an 'atomic'
 *   operation in the sense that any other I2C actions will be serialized
 *   and pend until this write completes.
 *
 * Input Parameters:
 *   dev    - Device-specific state data
 *   config  - Described the I2C configuration
 *   buffer - A pointer to the read-only buffer of data to be written to device
 *   buflen - The number of bytes to send from the buffer
 *
 * Returned Value:
 *   0: success, <0: A negated errno
 *
 ****************************************************************************/
//
//int hmc6343_dops_open(FAR void *instance_handle, int32_t arg)
//{
//	FAR struct hmc6343_dev_s *priv = (struct hmc6343_dev_s *)instance_handle;
//	struct i2c_msg_s msg;
//	uint8_t buff[3];
//	int ret = 0;
//
//
//	/* Setup device hmc6343 Operational Mode Register 1*/
//	buff[0] = HMC6343_REG_ACCELMAG_WRITE_EEPROM;
//	buff[1] = priv->config->initial_cr_values[OMR1].addr;
//	buff[2] = priv->config->initial_cr_values[OMR1].value;
//
//	msg.frequency = priv->config->frequency;
//	msg.addr      = priv->config->address;
//	msg.flags     = 0;
//	msg.buffer    = buff;
//	msg.length    = 4;
//
//	ret = priv->i2c->ops->transfer(priv->i2c,&msg, 1);
//	if(ret < 0){
//		snerr("ERROR: Failed to setup hmc6343 Operational Mode Register 1: %d\n", ret);
//		return ret;
//	}
//
//	usleep(10000);
//
////	/* Setup device hmc6343 Operational Mode Register 2*/
////	buff[0] = HMC6343_REG_ACCELMAG_WRITE_EEPROM;
////	buff[1] = priv->config->initial_cr_values[OMR2].addr;
////	buff[2] = priv->config->initial_cr_values[OMR2].value;
////
////	msg.frequency = priv->config->frequency;
////	msg.addr      = priv->config->address;
////	msg.flags     = 0;
////	msg.buffer    = buff;
////	msg.length    = 3;
////
////	ret = priv->i2c->ops->transfer(priv->i2c,&msg, 1);
////	if(ret < 0){
////		snerr("ERROR: Failed to setup hmc6343 Operational Mode Register 2: %d\n", ret);
////		return ret;
////	}
//
//	return ret;
//}

int hmc6343_dops_open(FAR void *instance_handle, int32_t arg)
{
	FAR struct hmc6343_dev_s *priv = (struct hmc6343_dev_s *)instance_handle;
	struct i2c_msg_s msg[2];
	uint8_t buff[3];
	int ret = 0;

	/* Setup device hmc6343 Operational Mode Register 1*/
	buff[0] = HMC6343_REG_ACCELMAG_WRITE_EEPROM;
	buff[1] = priv->config->initial_cr_values[OMR1].addr;
	buff[2] = priv->config->initial_cr_values[OMR1].value;

	msg[0].frequency = priv->config->frequency;
	msg[0].addr      = priv->config->address;
	msg[0].flags     = I2C_M_NOSTOP;
	msg[0].buffer    = buff;
	msg[0].length    = 2;

	msg[1].frequency = priv->config->frequency;
	msg[1].addr      = priv->config->address;
	msg[1].flags     = I2C_M_NOSTART;
	msg[1].buffer    = &buff[2];
	msg[1].length    = 1;

	ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
	if(ret < 0){
		snerr("ERROR: Failed to setup hmc6343 Operational Mode Register 1: %d\n", ret);
		return ret;
	}


	/* Setup device hmc6343 Operational Mode Register 2*/
	buff[0] = HMC6343_REG_ACCELMAG_WRITE_EEPROM;
	buff[1] = priv->config->initial_cr_values[OMR2].addr;
	buff[2] = priv->config->initial_cr_values[OMR2].value;

	msg[0].frequency = priv->config->frequency;
	msg[0].addr      = priv->config->address;
	msg[0].flags     = 0;
	msg[0].buffer    = buff;
	msg[0].length    = 2;

	msg[1].frequency = priv->config->frequency;
	msg[1].addr      = priv->config->address;
	msg[1].flags     = I2C_M_NOSTART;
	msg[1].buffer    = &buff[2];
	msg[1].length    = 1;


	ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
	if(ret < 0){
		snerr("ERROR: Failed to setup hmc6343 Operational Mode Register 2: %d\n", ret);
		return ret;
	}


	return ret;
}

//int hmc6343_dops_open(FAR void *instance_handle, int32_t arg)
//{
//	FAR struct hmc6343_dev_s *priv = (struct hmc6343_dev_s *)instance_handle;
//	struct i2c_msg_s msg[2];
//	int ret = 0;
//	uint8_t addr = 0;
//
//	/* Get device hmc6343 Software Version Number */
//	uint8_t buf[2];
//	buf[0] = HMC6343_REG_ACCELMAG_READ_EEPROM;
//	buf[1] = HMC6343_REG_I2C_SLAVE_ADDRESS;
//
//	msg[0].frequency = priv->config->frequency;
//	msg[0].addr      = priv->config->address;
//	msg[0].flags     = 0;
//	msg[0].buffer    = buf;
//	msg[0].length    = 2;
//
//	msg[1].frequency = priv->config->frequency;
//	msg[1].addr      = priv->config->address;
//	msg[1].flags     = I2C_M_READ;
//	msg[1].buffer    = &addr;
//	msg[1].length    = 1;
//
//	ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
//	if(ret < 0){
//		snerr("ERROR: Failed to read hmc6343 i2c address: %d\n", ret);
//		return ret;
//	}
//
//	if(addr != 0x32){
//		snerr("ERROR: Invalid i2c address:%d  board's ic address:%d\n",msg[0].addr, addr);
//	}
//
//	return  ret;
//}


CODE int hmc6343_dops_close(FAR void *instance_handle, int32_t arg)
{

	return OK;
}


CODE ssize_t hmc6343_dops_read(FAR void *instance_handle, FAR char *buffer,
          size_t buflen)
{
	return OK;
}


CODE ssize_t hmc6343_dops_write(FAR void *instance_handle,
          FAR const char *buffer, size_t buflen)
{
	return OK;
}

CODE off_t hmc6343_dops_seek(FAR void *instance_handle, off_t offset,
          int whence)
{
	return OK;
}

CODE int hmc6343_dops_ioctl(FAR void *instance_handle, int cmd,
          unsigned long arg)
{
	FAR struct hmc6343_dev_s *priv = (struct hmc6343_dev_s *)instance_handle;
	struct hmc6343_sample_s *data;
	struct i2c_msg_s msg[2];
	int ret = 0;

	switch(cmd){

	case HMC6343IOC_UPDATING_RATE_SET:
	{
		/* Set device hmc6343 Measurement Rate*/
		uint8_t buf[3];
		buf[0] = HMC6343_REG_ACCELMAG_WRITE_EEPROM;
		buf[1] = HMC6343_OMR1;
		buf[2] = (uint8_t)arg;

		msg[0].frequency = priv->config->frequency;
		msg[0].addr      = priv->config->address;
		msg[0].flags     = 0;
		msg[0].buffer    = buf;
		msg[0].length    = 3;

		ret = priv->i2c->ops->transfer(priv->i2c,msg, 1);
		if(ret < 0){
			snerr("ERROR: Failed to setup hmc6343 Measurement Rate: %d\n", ret);
			return ret;
		}
	}
		break;

	case HMC6343IOC_SOFTWARE_VERSION_GET:
	{
		/* Get device hmc6343 Software Version Number */
		uint8_t buf[2];
		buf[0] = HMC6343_REG_ACCELMAG_READ_EEPROM;
		buf[1] = HMC6343_REG_ACCELACCELMAG_SW_VERSION;

		msg[0].frequency = priv->config->frequency;
		msg[0].addr      = priv->config->address;
		msg[0].flags     = 0;
		msg[0].buffer    = buf;
		msg[0].length    = 2;

		msg[1].frequency = priv->config->frequency;
		msg[1].addr      = priv->config->address;
		msg[1].flags     = I2C_M_READ;
		msg[1].buffer    = (uint8_t *)arg;
		msg[1].length    = 1;

		ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
		if(ret < 0){
			snerr("ERROR: Failed to get hmc6343 software version Number: %d\n", ret);
			return ret;
		}
	}
		break;

	case HMC6343IOC_ACCELEROMETER_GET:
	{
		/* Get device hmc6343 accelerometer */
		uint8_t reg ,buff[6];
		reg = HMC6343_REG_ACCELMAG_POST_ACCEL;
		data = (struct hmc6343_sample_s *)arg;

		msg[0].frequency = priv->config->frequency;
		msg[0].addr      = priv->config->address;
		msg[0].flags     = 0;
		msg[0].buffer    = &reg;
		msg[0].length    = 1;

		msg[1].frequency = priv->config->frequency;
		msg[1].addr      = priv->config->address;
		msg[1].flags     = I2C_M_READ;
		msg[1].buffer    = (uint8_t *)buff;
		msg[1].length    = 6;

		ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
		if(ret < 0){
			snerr("ERROR: Failed to get hmc6343 accelerometer: %d\n", ret);
			return ret;
		}

		data->X = (int16_t)((buff[0] << 8) | buff[1]);
		data->Y = (int16_t)((buff[2] << 8) | buff[3]);
		data->Z = (int16_t)((buff[4] << 8) | buff[5]);
	}
		break;

	case HMC6343IOC_MAGNETOMETER_GET:
	{
		/* Get device hmc6343 magnetometer */
		uint8_t reg ,buff[6];
		reg = HMC6343_REG_ACCELMAG_POST_MAG;
		data = (struct hmc6343_sample_s *)arg;

		msg[0].frequency = priv->config->frequency;
		msg[0].addr      = priv->config->address;
		msg[0].flags     = 0;
		msg[0].buffer    = &reg;
		msg[0].length    = 1;

		msg[1].frequency = priv->config->frequency;
		msg[1].addr      = priv->config->address;
		msg[1].flags     = I2C_M_READ;
		msg[1].buffer    = (uint8_t *)buff;
		msg[1].length    = 6;

		ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
		if(ret < 0){
			snerr("ERROR: Failed to get hmc6343 magnetometer: %d\n", ret);
			return ret;
		}

		data->X = (int16_t)((buff[0] << 8) | buff[1]);
		data->Y = (int16_t)((buff[2] << 8) | buff[3]);
		data->Z = (int16_t)((buff[4] << 8) | buff[5]);
	}
		break;

	case HMC6343IOC_HEADING_DATA_GET:
	{
		/* Get device hmc6343 heading data */
		uint8_t reg ,buff[6];
		reg = HMC6343_REG_ACCELMAG_POST_HEADING;
		data = (struct hmc6343_sample_s *)arg;

		msg[0].frequency = priv->config->frequency;
		msg[0].addr      = priv->config->address;
		msg[0].flags     = 0;
		msg[0].buffer    = &reg;
		msg[0].length    = 1;

		msg[1].frequency = priv->config->frequency;
		msg[1].addr      = priv->config->address;
		msg[1].flags     = I2C_M_READ;
		msg[1].buffer    = (uint8_t *)buff;
		msg[1].length    = 6;

		ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
		if(ret < 0){
			snerr("ERROR: Failed to get hmc6343 heading: %d\n", ret);
			return ret;
		}

		data->X = (int16_t)((buff[0] << 8) | buff[1]);
		data->Y = (int16_t)((buff[2] << 8) | buff[3]);
		data->Z = (int16_t)((buff[4] << 8) | buff[5]);
	}
		break;

	case HMC6343IOC_TILT_DATA_GET:
	{
		/* Get device hmc6343 tilt data */
		uint8_t reg ,buff[6];
		reg = HMC6343_REG_ACCELMAG_POST_TILT;
		data = (struct hmc6343_sample_s *)arg;

		msg[0].frequency = priv->config->frequency;
		msg[0].addr      = priv->config->address;
		msg[0].flags     = 0;
		msg[0].buffer    = &reg;
		msg[0].length    = 1;

		msg[1].frequency = priv->config->frequency;
		msg[1].addr      = priv->config->address;
		msg[1].flags     = I2C_M_READ;
		msg[1].buffer    = (uint8_t *)buff;
		msg[1].length    = 6;

		ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
		if(ret < 0){
			snerr("ERROR: Failed to get hmc6343 heading: %d\n", ret);
			return ret;
		}

		data->X = (int16_t)((buff[0] << 8) | buff[1]);
		data->Y = (int16_t)((buff[2] << 8) | buff[3]);
		data->Z = (int16_t)((buff[4] << 8) | buff[5]);
	}
		break;

	case HMC6343IOC_SERIAL_NUMBER_GET:
		{
			/* Get device hmc6343 Serial Number */
			uint8_t buf[2],sn[2];
			buf[0] = HMC6343_REG_ACCELMAG_READ_EEPROM;
			buf[1] = HMC6343_REG_ACCELMAG_SN_LSB;

			msg[0].frequency = priv->config->frequency;
			msg[0].addr      = priv->config->address;
			msg[0].flags     = 0;
			msg[0].buffer    = buf;
			msg[0].length    = 2;

			msg[1].frequency = priv->config->frequency;
			msg[1].addr      = priv->config->address;
			msg[1].flags     = I2C_M_READ;
			msg[1].buffer    = &sn[0];
			msg[1].length    = 1;

			ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
			if(ret < 0){
				snerr("ERROR: Failed to get hmc6343 Serial Number LSB: %d\n", ret);
				return ret;
			}

			buf[0] = HMC6343_REG_ACCELMAG_READ_EEPROM;
			buf[1] = HMC6343_REG_ACCELMAG_SN_MSB;

			msg[0].frequency = priv->config->frequency;
			msg[0].addr      = priv->config->address;
			msg[0].flags     = 0;
			msg[0].buffer    = buf;
			msg[0].length    = 2;

			msg[1].frequency = priv->config->frequency;
			msg[1].addr      = priv->config->address;
			msg[1].flags     = I2C_M_READ;
			msg[1].buffer    = &sn[1];
			msg[1].length    = 1;

			ret = priv->i2c->ops->transfer(priv->i2c,msg, 2);
			if(ret < 0){
				snerr("ERROR: Failed to get hmc6343 Serial Number MSB: %d\n", ret);
				return ret;
			}

			*(uint16_t *)arg = (uint16_t)((sn[0] << 8) | sn[1]);
		}
			break;

	case HMC6343IOC_MEASUREMENT_RATE_SET:
		{
			priv->_mesure_ticks = USEC2TICK(arg);
			ret = OK;
		}
		break;

	case HMC6343IOC_MEASUREMENT_START:
		{
			hmc6343_start_cycle(priv);
			ret = OK;
		}
		break;

	case HMC6343IOC_MEASUREMENT_STOP:
		{
			hmc6343_stop_cycle(priv);
			ret = OK;
		}
		break;

	default:

		break;
	}

	return ret;
}

#endif /* CONFIG_SENSORS_HMC6343 && CONFIG_I2C*/
