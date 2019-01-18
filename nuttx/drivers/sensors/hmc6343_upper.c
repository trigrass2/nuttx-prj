/****************************************************************************
 * drivers/sensors/hmc6343_upper.c
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


/****************************************************************************
 * Private Types
 ****************************************************************************/
#define DEV_FORMAT   "/dev/hmc6343_%d"
#define DEV_NAMELEN  16

/****************************************************************************
 * Private Functions
 ****************************************************************************/
extern int 		hmc6343_dops_open(FAR void *instance_handle, int32_t arg);
extern int 		hmc6343_dops_close(FAR void *instance_handle, int32_t arg);
extern ssize_t 	hmc6343_dops_read(FAR void *instance_handle, FAR char *buffer,size_t buflen);
extern ssize_t	hmc6343_dops_write(FAR void *instance_handle,FAR const char *buffer, size_t buflen);
extern int 		hmc6343_dops_ioctl(FAR void *instance_handle, int cmd,unsigned long arg);



/* Character driver methods */

static int     hmc6343_open(FAR struct file *filep);
static int     hmc6343_close(FAR struct file *filep);
static ssize_t hmc6343_read(FAR struct file *filep, FAR char *buffer,\
							size_t len);
static ssize_t hmc6343_write(FAR struct file *filep, FAR const char *buffer,\
        					size_t buflen);
static off_t   hmc6343_seek(FAR struct file *filep, off_t offset,\
							int whence);
static int 	   hmc6343_ioctl(FAR struct file *filep, int cmd,\
							unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct hmc6343_reg_pair_s g_hmc6343_cr_vals[] =
{
	// Operational Mode Register 1
	{HMC6343_OMR1,HMC6343_OMR1_LEVEL|HMC6343_OMR1_RUN|HMC6343_OMR1_IIR_FILTER},

	// Operational Mode Register 2
	{HMC6343_OMR2,HMC6343_OMR2_MR_10HZ}
};

struct sensor_cluster_operations_s g_sc_ops =
{
	.driver_open  	= hmc6343_dops_open,
	.driver_close 	= hmc6343_dops_close,
	.driver_read  	= hmc6343_dops_read,
	.driver_write 	= hmc6343_dops_write,
	.driver_ioctl 	= hmc6343_dops_ioctl,
	.driver_seek  	= NULL,
	.driver_suspend = NULL,
	.driver_resume 	= NULL
};

struct hmc6343_config_s g_hmc6343cfgs =
{
	.address   			= HMC6343_I2C_DEFAULT_ADDRESS,		/* i2c default address */
	.frequency 			= HMC6343_I2C_STANDARD_FREQUENCY,	/* i2c default frequency */
	.initial_cr_values 	= g_hmc6343_cr_vals,
	.sc_ops 			= &g_sc_ops
};

/* This the vtable that supports the character driver interface */

static const struct file_operations g_hmc6343fops =
{
  hmc6343_open,    /* open */
  hmc6343_close,   /* close */
  hmc6343_read,    /* read */
  hmc6343_write,   /* write */
  hmc6343_seek,    /* seek */
  hmc6343_ioctl,   /* ioctl */
};

/****************************************************************************
 * Name: hmc6343_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int hmc6343_open(FAR struct file *filep)
{
	  FAR struct inode *inode = filep->f_inode;
	  FAR struct hmc6343_dev_s *priv = inode->i_private;
	  int ret;
	  uint8_t tmp;


	  /* If the port is the middle of closing, wait until the close is finished */

	  /* Take a count from the semaphore, possibly waiting */

	  ret = nxsem_wait(&priv->exclsem);
	  if (ret < 0)
	    {
	      return ret;
	    }

	  /* Increment the count of references to the device.  If this is the first
	   * time that the driver has been opened for this device, then initialize
	   * the device.
	   */

	  tmp = priv->crefs + 1;
	  if (tmp > 1)
	    {
	      /* More than 1 opens; uint8_t overflows to zero */

	      ret = -EMFILE;
	    }
	  else
	    {
	      /* Check if this is the first time that the driver has been opened. */

	      if (tmp == 1){
	          /* Yes.. perform one time hardware initialization. */

	          ret = priv->config->sc_ops->driver_open((FAR void *)priv, 0);
	          if (ret < 0){
	        	  snwarn("WARN: Failed to open HMC6343: %d\n", ret);
	            }

	        }
	        /* Save the incremented open count */

	        priv->crefs = tmp;
	    }

	  /* Release the sensor */

	  nxsem_post(&priv->exclsem);

	  return ret;
}

/****************************************************************************
 * Name: hmc6343_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int hmc6343_close(FAR struct file *filep)
{
	  FAR struct inode *inode = filep->f_inode;
	  FAR struct hmc6343_dev_s *priv = inode->i_private;
	  int ret;

	  ret = nxsem_wait(&priv->exclsem);
	  if (ret < 0){
	      return ret;
	  }

	  /* Decrement the references to the driver.  If the reference count will
	   * decrement to 0, then uninitialize the driver.
	   */

	  if (priv->crefs > 1){

		  priv->crefs--;

		  /* Release the sensor */

		  nxsem_post(&priv->exclsem);

		  return ret;
	  }

	  /* There are no more references to the port */

	  priv->crefs = 0;

	  /* Close the device */

	  ret = priv->config->sc_ops->driver_close((FAR void *)priv, 0);

	  /* Release the sensor */

	  nxsem_post(&priv->exclsem);

	  return ret;
}

/****************************************************************************
 * Name: hmc6343_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t hmc6343_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hmc6343_dev_s *priv = inode->i_private;

  return priv->config->sc_ops->driver_read(priv, buffer, len);
}

/****************************************************************************
 * Name: hmc6343_write
 *
 * Description:
 *   Standard character driver write method.
 *
 ****************************************************************************/

static ssize_t hmc6343_write(FAR struct file *filep, FAR const char *buffer,
        					size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hmc6343_dev_s *priv = inode->i_private;

  return priv->config->sc_ops->driver_write(priv, buffer, buflen);
}

/****************************************************************************
 * Name: adis16488_seek
 ****************************************************************************/

static off_t hmc6343_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hmc6343_dev_s *priv = inode->i_private;

  return priv->config->sc_ops->driver_seek(priv, offset, whence);
}

/****************************************************************************
 * Name: adis16488_ioctl
 ****************************************************************************/

static int hmc6343_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hmc6343_dev_s *priv = inode->i_private;

  return priv->config->sc_ops->driver_ioctl(priv, cmd, arg);
}


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

int hmc6343_register(FAR struct i2c_master_s *dev, int minor)
{
	FAR struct hmc6343_dev_s *priv;
	char devname[DEV_NAMELEN];
	int ret;

	/* Allocate the HMC6343 driver instance */

	priv = (FAR struct hmc6343_dev_s *)kmm_zalloc(sizeof(struct hmc6343_dev_s));
	if (!priv)
	{
	  snerr("ERROR: Failed to allocate the device structure!\n");
	  return ERROR;
	}

	/* Initialize the device state structure */

	priv->config = &g_hmc6343cfgs;
	priv->i2c 	 = dev;

	nxsem_init(&priv->exclsem, 0, 1);
	nxsem_init(&priv->waitsem, 0, 1);

	DEBUGASSERT(priv);

	/* Get exclusive access to the device structure */

	ret = nxsem_wait(&priv->exclsem);
	if (ret < 0)
	{
	  snerr("ERROR: nxsem_wait failed: %d\n", ret);
	  DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
	  return ret;
	}

  /* Initialize the structure fields to their default values */

	priv->ofsx = 0;
	priv->ofsy = 0;
	priv->ofsz = 0;

	priv->sample.X = 0;
	priv->sample.Y = 0;
	priv->sample.Z = 0;

	priv->_mesure_ticks = USEC2TICK(50000);
	priv->_running		= false;

  /* Register the character driver */

  snprintf(devname, DEV_NAMELEN, DEV_FORMAT, minor);
  ret = register_driver(devname, &g_hmc6343fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver %s: %d\n", devname, ret);
      nxsem_post(&priv->exclsem);
      return ret;
    }

  /* Indicate that the accelerometer was successfully initialized */

  priv->status |= HMC6343_STAT_INITIALIZED;  /* compass is initialized */

  nxsem_post(&priv->exclsem);

  return ret;
}

/****************************************************************************
 * Name: hmc6343_worker
 *
 * Description:
 *   This is the "bottom half" of the HMC6343 interrupt handler
 *
 ****************************************************************************/

// static void hmc6343_worker(FAR void *arg)
// {
//  FAR struct hmc6343_dev_s *priv = (FAR struct hmc6343_dev_s *)arg;
//  uint8_t regval;
//
//  DEBUGASSERT(priv && priv->config);
//
//  /* Get the global interrupt status */
//
//  regval =  hmc6343_getreg8(priv, HMC6343_INT_SOURCE);
//
//  /* Check for a data ready interrupt */
//
//  if ((regval & INT_DATA_READY) != 0)
//    {
//      /* Read accelerometer data to sample */
//
//      priv->sample.data_x =  hmc6343_getreg8(priv, HMC6343_DATAX1);
//      priv->sample.data_x = (priv->sample.data_x << 8) | hmc6343_getreg8(priv, HMC6343_DATAX0);
//      priv->sample.data_y =  hmc6343_getreg8(priv, HMC6343_DATAY1);
//      priv->sample.data_y = (priv->sample.data_y << 8) | hmc6343_getreg8(priv, HMC6343_DATAY0);
//      priv->sample.data_z =  hmc6343_getreg8(priv, HMC6343_DATAZ1);
//      priv->sample.data_z = (priv->sample.data_z << 8) | hmc6343_getreg8(priv, HMC6343_DATAZ0);
//    }
//
//  /* Re-enable the HMC6343 GPIO interrupt */
//
//  priv->config->enable(priv->config, true);
// }

/****************************************************************************
 * Name: hmc6343_interrupt
 *
 * Description:
 *  The HMC6343 interrupt handler
 *
 ****************************************************************************/

// static void hmc6343_interrupt(FAR struct hmc6343_config_s *config, FAR void *arg)
// {
//  FAR struct hmc6343_dev_s *priv = (FAR struct hmc6343_dev_s *)arg;
//  int ret;
//
//  DEBUGASSERT(priv && priv->config == config);
//
//  /* Disable further interrupts */
//
//  config->enable(config, false);
//
//  /* Check if interrupt work is already queue.  If it is already busy, then
//   * we already have interrupt processing in the pipeline and we need to do
//   * nothing more.
//   */
//
//  if (work_available(&priv->work))
//    {
//      /* Yes.. Transfer processing to the worker thread.  Since HMC6343
//       * interrupts are disabled while the work is pending, no special
//       * action should be required to protect the work queue.
//       */
//
//      ret = work_queue(HPWORK, &priv->work, hmc6343_worker, priv, 0);
//      if (ret != 0)
//        {
//          snerr("ERROR: Failed to queue work: %d\n", ret);
//        }
//    }
//
//  /* Clear any pending interrupts and return success */
//
//  config->clear(config);
// }


#endif /* CONFIG_SENSORS_HMC6343 && CONFIG_I2C*/
