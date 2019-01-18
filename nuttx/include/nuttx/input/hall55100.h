/****************************************************************************
 * drivers/input/hall55100.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_HALLS_H
#define __INCLUDE_NUTTX_HALLS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/fs/ioctl.h>
#include <stdbool.h>
#include <sys/types.h>

#ifdef CONFIG_HALLS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOCTL Commands ***********************************************************/
/* The hall driver uses a standard character driver framework.  However,
 * since the hall driver is a device control interface and not a data
 * transfer interface, the majority of the functionality is implemented in
 * driver ioctl calls.  The hall ioctl commands are listed below:
 *
 * These are detected and handled by the "upper half" hall driver.
 *
 * HALLIOC_NOTIFICATION - Set up to notify an application via a signal when
 *                      the hall expires.
 *                      Argument: A read-only pointer to an instance of
 *                      stuct hall_notify_s.
 *
 * WARNING: May change HALLIOC_SETTIMEOUT to pass pointer to 64bit nanoseconds
 * or timespec structure.
 *
 * NOTE: The HALLIOC_SETHANDLER ioctl cannot be supported in the kernel build
 * mode. In that case direct callbacks from kernel space into user space is
 * forbidden.
 *
 * NOTE: _HALLIOC(0x0001) througn _HALLIOC(0x001f) are reserved for use by the
 * hall driver to assure that the values are unique.  Other hall drivers,
 * such as the oneshot hall, must not use IOCTL commands in this numeric
 * range.
 */

#define HALLIOC_SETUP 			0x0001
#define HALLIOC_CLOSE		 	0x0002
#define HALLIOC_ENABLE        	0x0003
#define HALLIOC_DISABLE        	0x0004
#define HALLIOC_SETCALLBACK 	0x0005



/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure provides the "lower-half" driver operations available to
 * the "upper-half" driver.
 */

struct hall_lowerhalf_s;
struct hall_ops_s
{
  /* Required methods ********************************************************/

  /* Start the timer, resetting the time to the current timeout */

  CODE int (*start)(FAR struct hall_lowerhalf_s *lower);

  /* Stop the timer */

  CODE int (*stop)(FAR struct hall_lowerhalf_s *lower);

  /* Any ioctl commands that are not recognized by the "upper-half" driver
   * are forwarded to the lower half driver through this method.
   */

  CODE int (*ioctl)(FAR struct hall_lowerhalf_s *lower, int cmd,
                    unsigned long arg);

  /* Call the NuttX INTERNAL timeout callback on timeout.
   * NOTE:  Providing callback==NULL disable.
   * NOT to call back into applications.
   */

  CODE void (*setcallback)(FAR struct hall_lowerhalf_s *lower,
                           CODE xcpt_t callback, FAR void *arg);


};

/* This structure provides the publicly visible representation of the
 * "lower-half" driver state structure.  "lower half" drivers will have an
 * internal structure definition that will be cast-compatible with this
 * structure definitions.
 */

struct hall_lowerhalf_s
{
  /* Publicly visible portion of the "lower-half" driver state structure. */

  FAR const struct hall_ops_s  *ops;  /* Lower half operations */

  FAR xcpt_t handle;

  FAR uint32_t pin;

  FAR bool risingedge;

  FAR bool fallingedge;

  FAR bool event;


};


/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * "Upper-Half" hall Driver Interfaces
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
 *   NOTE:  Normally, this function would not be called by application code.
 *   Rather it is called indirectly through the architecture-specific
 *   initialization.
 *
 * Input Parameters:
 *   dev path - The full path to the driver to be registers in the NuttX
 *     pseudo-filesystem.  The recommended convention is to name all hall
 *     drivers as "/dev/hall0", "/dev/hall1", etc.  where the driver
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
                         FAR struct hall_lowerhalf_s *lower);

/****************************************************************************
 * Name: hall_unregister
 *
 * Description:
 *   This function can be called to disable and unregister the hall
 *   device driver.
 *
 * Input Parameters:
 *   handle - This is the handle that was returned by hall_register()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void hall_unregister(FAR void *handle);

/****************************************************************************
 * Kernal internal interfaces.  Thse may not be used by application logic
 ****************************************************************************/


/****************************************************************************
 * Platform-Independent "Lower-Half" hall Driver Interfaces
 ****************************************************************************/

/****************************************************************************
 * Architecture-specific Application Interfaces
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_HALLS */
#endif  /* __INCLUDE_NUTTX_HALLS_H */

