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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/adis16488.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Private structure definitions
 ****************************************************************************/



/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      adis16488_open(FAR struct file *filep);
static int      adis16488_close(FAR struct file *filep);
static ssize_t  adis16488_read(FAR struct file *, FAR char *, size_t);
static ssize_t  adis16488_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen);
static off_t    adis16488_seek(FAR struct file *filep, off_t offset,
                             int whence);
static int      adis16488_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_adis16488_fops =
{
  adis16488_open,
  adis16488_close,
  adis16488_read,
  adis16488_write,
  adis16488_seek,
  adis16488_ioctl
#ifndef CONFIG_DISABLE_POLL
  , NULL
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL
#endif
};


/****************************************************************************
 * Private data storage
 ****************************************************************************/



/****************************************************************************
 * Name: adis16488_open
 ****************************************************************************/

 static int adis16488_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adis16488_dev_s *priv = inode->i_private;
  int ret;
  uint8_t tmp;


  /* If the port is the middle of closing, wait until the close is finished */

  /* Take a count from the semaphore, possibly waiting */

  ret = nxsem_wait(&priv->devicesem);
  if (ret < 0)
    {
      return ret;
    }

  /* Increment the count of references to the device.  If this is the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = priv->cd_ocount + 1;
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

          ret = priv->config->sc_ops->c.driver_open((FAR void *)priv, 0);
          if (ret < 0){
        	  snwarn("WARN: Failed to open ADIS16488: %d\n", ret);
            }

        }
        /* Save the incremented open count */

        priv->cd_ocount = tmp;
    }

  /* Release the sensor */

  nxsem_post(&priv->devicesem);

  return ret;
}

/****************************************************************************
 * Name: adis16488_close
 ****************************************************************************/

static int adis16488_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adis16488_dev_s *priv = inode->i_private;
  int ret;

  ret = nxsem_wait(&priv->devicesem);
  if (ret < 0){
      return ret;
  }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (priv->cd_ocount > 1){

	  priv->cd_ocount--;

	  /* Release the sensor */

	  nxsem_post(&priv->devicesem);

	  return ret;
  }

  /* There are no more references to the port */

  priv->cd_ocount = 0;

  /* Close the device */

  ret = priv->config->sc_ops->c.driver_close((FAR void *)priv, 0);

  /* Release the sensor */

  nxsem_post(&priv->devicesem);

  return ret;

}

/****************************************************************************
 * Name: adis16488_read
 ****************************************************************************/

static ssize_t adis16488_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adis16488_dev_s *priv = inode->i_private;

  return priv->config->sc_ops->c.driver_read(priv, buffer, buflen);
}

/****************************************************************************
 * Name: adis16488_write
 ****************************************************************************/

static ssize_t adis16488_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adis16488_dev_s *priv = inode->i_private;

  return priv->config->sc_ops->c.driver_write(priv, buffer, buflen);
}
/****************************************************************************
 * Name: adis16488_seek
 ****************************************************************************/

static off_t adis16488_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adis16488_dev_s *priv = inode->i_private;

  return priv->config->sc_ops->c.driver_seek(priv, offset, whence);
}

/****************************************************************************
 * Name: adis16488_ioctl
 ****************************************************************************/

static int adis16488_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct adis16488_dev_s *priv = inode->i_private;

  return priv->config->sc_ops->c.driver_ioctl(priv, cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adis16488_register
 *
 * Description:
 *   Register the ADIS16488 character device as 'devpath'
 *
 * Input Parameters:
 *   spi      - An instance of the SPI interface to use to communicate with
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int adis16488_register(FAR struct spi_dev_s *spi,FAR int num)
{
  FAR struct adis16488_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the ADIS16488 accelerometer device structure. */

  priv = (FAR struct adis16488_dev_s *)kmm_malloc(sizeof(struct adis16488_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate accelerometer instance\n");
      return -ENOMEM;
    }

  switch(num)
  {
  case 1:
	  {
		  priv->spi           	= spi;
		  priv->config        	= adis16488_spi_config_initialize(1);
		  priv->cd_ocount		= 0;
		  priv->seek_address	= 0;
		  priv->readonly		= 0;
		  //work queue
		  priv->_mesure_ticks		= USEC2TICK(4000);
		  priv->_running			= false;

		  /* Initialize sensor and sensor data access semaphore */

		  nxsem_init(&priv->devicesem, 0, 1);
		  nxsem_init(&priv->datasem, 0, 1);

		  /* Register the character driver */

		  ret = register_driver("/dev/adis16488_1", &g_adis16488_fops, 0666, priv);
		  if (ret < 0)
			{
			  snerr("ERROR: Failed to register accelerometer driver: %d\n", ret);

			  nxsem_destroy(&priv->datasem);
			  kmm_free(priv);
			  return ret;
			}

	  }
	  break;
  default:
	  return -ENODEV;
	  break;


  }

  return OK;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_ADIS16488 && CONFIG_SPI_EXCHANGE */
