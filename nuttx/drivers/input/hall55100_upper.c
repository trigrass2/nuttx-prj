/****************************************************************************
 * drivers/input/hall55100.c
 *
 *   Copyright (C) 2014, 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>

#include <nuttx/input/hall55100.h>

#ifdef CONFIG_HALLS

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct hall_upperhalf_s
{
  uint8_t   crefs;         /* The number of times the device has been opened */
  FAR void *arg;           /* An argument to pass with the signal */
  FAR char *path;          /* Registration path */

  /* The contained lower-half driver */

  FAR struct hall_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     hall_open(FAR struct file *filep);
static int     hall_close(FAR struct file *filep);
static ssize_t hall_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t hall_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     hall_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_hallops =
{
  hall_open,  /* open */
  hall_close, /* close */
  hall_read,  /* read */
  hall_write, /* write */
  NULL,        /* seek */
  hall_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , NULL       /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL       /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/************************************************************************************
 * Name: hall_open
 *
 * Description:
 *   This function is called whenever the hall device is opened.
 *
 ************************************************************************************/

static int hall_open(FAR struct file *filep)
{
  FAR struct inode             *inode = filep->f_inode;
  FAR struct hall_upperhalf_s  *upper = inode->i_private;
  FAR struct hall_lowerhalf_s  *lower = upper->lower;
  uint8_t                       tmp;
  int                           ret;

  tmrinfo("crefs: %d\n", upper->crefs);

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout;
    }
  else
	  {
		/* Check if this is the first time that the driver has been opened. */

		if (tmp == 1)
		  {
			/* Yes.. perform one time hardware initialization. */

			irqstate_t flags = enter_critical_section();

			lower->ops->start(lower);

			leave_critical_section(flags);
		  }
	  }

  /* Save the new open count */

  upper->crefs = tmp;

  ret = OK;

errout:
  return ret;
}

/************************************************************************************
 * Name: hall_close
 *
 * Description:
 *   This function is called when the hall device is closed.
 *
 ************************************************************************************/

static int hall_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct hall_upperhalf_s *upper = inode->i_private;

  tmrinfo("crefs: %d\n", upper->crefs);

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 0){
      upper->crefs--;
    }

  return OK;
}

/************************************************************************************
 * Name: hall_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t hall_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/************************************************************************************
 * Name: hall_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t hall_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return 0;
}

/************************************************************************************
 * Name: hall_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the hall work is
 *   done.
 *
 ************************************************************************************/

static int hall_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode             *inode = filep->f_inode;
  FAR struct hall_upperhalf_s  *upper;
  FAR struct hall_lowerhalf_s  *lower;
  int                           ret;

  tmrinfo("cmd: %d arg: %ld\n", cmd, arg);
  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
    /* cmd:         HALLIOC_ENABLE
     * Description: Start the hall
     * Argument:    Ignored
     */

    case HALLIOC_SETUP:
      {
        /* Start the hall, resetting the time to the current timeout */

        if (lower->ops->start)
          {
            ret = lower->ops->start(lower);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* cmd:         HALLIOC_DISABLE
     * Description: Stop the hall
     * Argument:    Ignored
     */

    case HALLIOC_CLOSE:
      {
        /* Stop the hall */

        if (lower->ops->stop)
          {
            ret = lower->ops->stop(lower);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;


    /* cmd:         HALLIOC_SETCALLBACK
     * Description: Notify application via a signal when the hall expires.
     * Argument:    signal number
     *
     * NOTE: This ioctl cannot be support in the kernel build mode. In that
     * case direct callbacks from kernel space into user space is forbidden.
     */

    case HALLIOC_SETCALLBACK:
      {
        FAR  xcpt_t cb =
          (FAR  xcpt_t )((uintptr_t)arg);

        if (cb != NULL)
          {

        	 lower->ops->setcallback(lower,cb,NULL);
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

    default:
      {
        /* An ioctl commands that are not recognized by the "upper-half"
         * driver are forwarded to the lower half driver through this
         * method.
         */

        if (lower->ops->ioctl) /* Optional */
          {
            ret = lower->ops->ioctl(lower, cmd, arg);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hall_register
 *
 * Description:
 *   This function binds an instance of a "lower half" hall driver with the
 *   "upper half" hall device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   disabled state (as if the stop() method had already been called).
 *
 * Input Parameters:
 *   dev path - The full path to the driver to be registers in the NuttX
 *     pseudo-filesystem.  The recommended convention is to name all hall
 *     drivers as "/dev/tc0", "/dev/tc1", etc.  where the driver
 *     path differs only in the "minor" number at the end of the device name.
 *   lower - A pointer to an instance of lower half hall driver.  This
 *     instance is bound to the hall driver and must persists as long as
 *     the driver persists.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned to the caller.  In the event
 *   of any failure, a NULL value is returned.
 *
 ****************************************************************************/

FAR void *hall_register(FAR const char *path,
                         FAR struct hall_lowerhalf_s *lower)
{
  FAR struct hall_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(path && lower);

  /* Allocate the upper-half data structure */

  upper = (FAR struct hall_upperhalf_s *)
    kmm_zalloc(sizeof(struct hall_upperhalf_s));
  if (!upper)
    {
      tmrerr("ERROR: Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the hall device structure (it was already zeroed
   * by kmm_zalloc()).
   */

  upper->lower = lower;

  /* Copy the registration path */

  upper->path = strdup(path);
  if (!upper->path)
    {
      tmrerr("ERROR: Path allocation failed\n");
      goto errout_with_upper;
    }

  /* Register the hall device */

  ret = register_driver(path, &g_hallops, 0666, upper);
  if (ret < 0)
    {
      tmrerr("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

  return (FAR void *)upper;

errout_with_path:
  kmm_free(upper->path);

errout_with_upper:
  kmm_free(upper);

errout:
  return NULL;
}


#endif /* CONFIG_HALLS */

