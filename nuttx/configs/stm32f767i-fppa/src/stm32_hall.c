/****************************************************************************
 * configs/stm32f429i-disco/src/stm32_halls.c
 *
 *   Copyright (C) 2011-2012, 2014-2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdint.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32f767i-fppa.h"
#include "stm32_gpio.h"

#include <nuttx/input/hall55100.h>

#ifdef CONFIG_HALLS

int hall_start(FAR struct hall_lowerhalf_s *lower);
int hall_stop(FAR struct hall_lowerhalf_s *lower);
int hall_ioctl(FAR struct hall_lowerhalf_s *lower, int cmd,
                  unsigned long arg);
void hall_setcallback(FAR struct hall_lowerhalf_s *lower,
                         CODE xcpt_t callback, FAR void *arg);
int hall_handle_default(int irq, FAR void *context, FAR void *arg);


/****************************************************************************
 * Private Data
 ****************************************************************************/
struct hall_ops_s g_hall_ops =
{
	.start 		 = hall_start,
	.stop 		 = hall_stop,
	.ioctl 		 = hall_ioctl,
	.setcallback = hall_setcallback
};


/*
 * Pin configuration for each hall.
 */
#ifdef CONFIG_HALL_DEV1
static struct hall_lowerhalf_s g_hall1_lower =
{
	.ops 		 = &g_hall_ops,
	.handle 	 = hall_handle_default,
	.pin 		 = GPIO_HW_RXIN1,
#ifdef CONFIG_HALL_DEV1_RISING_EDGE
	.risingedge  = true,
#else
	.risingedge  = false,
#endif

#ifdef CONFIG_HALL_DEV1_FALLING_EDGE
	.fallingedge  = true,
#else
	.fallingedge  = false,
#endif
	.event       = true
};
#endif //CONFIG_HALL_DEV1

#ifdef CONFIG_HALL_DEV2
static struct hall_lowerhalf_s g_hall2_lower =
{
	.ops 		 = &g_hall_ops,
	.handle 	 = hall_handle_default,
	.pin 		 = GPIO_HW_RXIN2,
#ifdef CONFIG_HALL_DEV2_RISING_EDGE
	.risingedge  = true,
#else
	.risingedge  = false,
#endif

#ifdef CONFIG_HALL_DEV2_FALLING_EDGE
	.fallingedge  = true,
#else
	.fallingedge  = false,
#endif
	.event       = true
};
#endif //CONFIG_HALL_DEV2

#ifdef CONFIG_HALL_DEV3
static struct hall_lowerhalf_s g_hall3_lower =
{
	.ops 	     = &g_hall_ops,
	.handle      = hall_handle_default,
	.pin 	     = GPIO_HW_RXIN3,
#ifdef CONFIG_HALL_DEV3_RISING_EDGE
	.risingedge  = true,
#else
	.risingedge  = false,
#endif

#ifdef CONFIG_HALL_DEV3_FALLING_EDGE
	.fallingedge  = true,
#else
	.fallingedge  = false,
#endif
	.event       = true
};
#endif //CONFIG_HALL_DEV1

#ifdef CONFIG_HALL_DEV4
static struct hall_lowerhalf_s g_hall4_lower =
{
	.ops 		 = &g_hall_ops,
	.handle 	 = hall_handle_default,
	.pin 		 = GPIO_HW_RXIN4,
#ifdef CONFIG_HALL_DEV4_RISING_EDGE
	.risingedge  = true,
#else
	.risingedge  = false,
#endif

#ifdef CONFIG_HALL_DEV4_FALLING_EDGE
	.fallingedge  = true,
#else
	.fallingedge  = false,
#endif
	.event       = true
};
#endif //CONFIG_HALL_DEV4

#ifdef CONFIG_HALL_DEV5
static struct hall_lowerhalf_s g_hall5_lower =
{
	.ops 		 = &g_hall_ops,
	.handle 	 = hall_handle_default,
	.pin 		 = GPIO_HW_RXIN5,
#ifdef CONFIG_HALL_DEV5_RISING_EDGE
	.risingedge  = true,
#else
	.risingedge  = false,
#endif

#ifdef CONFIG_HALL_DEV5_FALLING_EDGE
	.fallingedge  = true,
#else
	.fallingedge  = false,
#endif
	.event       = true
};
#endif //CONFIG_HALL_DEV5


#ifdef CONFIG_HALL_DEV6
static struct hall_lowerhalf_s g_hall6_lower =
{
	.ops 		 = &g_hall_ops,
	.handle 	 = hall_handle_default,
	.pin 		 = GPIO_HW_RXIN6,
#ifdef CONFIG_HALL_DEV6_RISING_EDGE
	.risingedge  = true,
#else
	.risingedge  = false,
#endif

#ifdef CONFIG_HALL_DEV6_FALLING_EDGE
	.fallingedge  = true,
#else
	.fallingedge  = false,
#endif
	.event       = true
};
#endif //CONFIG_HALL_DEV6




/****************************************************************************
 * Public Functions
 ****************************************************************************/

int hall_handle_default(int irq, FAR void *context, FAR void *arg)
{
	return 1;
}


/* Start the hall */

int hall_start(FAR struct hall_lowerhalf_s *lower)
{
	int ret = -EINVAL;

    ret = stm32_configgpio(lower->pin);
    if(ret < 0 ){
    	return ret;
    }

    ret = stm32_gpiosetevent(lower->pin, lower->risingedge, lower->fallingedge, lower->event, lower->handle, NULL);
    if(ret < 0 ){
    	return ret;
    }

    return OK;
}

/* Stop the timer */

int hall_stop(FAR struct hall_lowerhalf_s *lower)
{
	return 0;
}

/* Any ioctl commands that are not recognized by the "upper-half" driver
 * are forwarded to the lower half driver through this method.
 */

int hall_ioctl(FAR struct hall_lowerhalf_s *lower, int cmd,
                  unsigned long arg)
{
	int ret = -EINVAL;
	switch(cmd)
	{
		/* enable interrupt */
		case HALLIOC_ENABLE:
		{
			ret = stm32_gpiosetevent(lower->pin, lower->risingedge, lower->fallingedge, true, lower->handle, NULL);
			if(ret < 0 ){
				return ret;
			}
		}
		break;

		/* disable interrupt */
		case HALLIOC_DISABLE:
		{
			ret = stm32_gpiosetevent(lower->pin, lower->risingedge, lower->fallingedge, false, lower->handle, NULL);
			if(ret < 0 ){
				return ret;
			}
		}
		break;

		default:
			return -EINVAL;
		break;
	}
	return 0;
}

/* Call the NuttX INTERNAL exit callback.
 * NOTE:  Providing callback==NULL disable.
 * NOT to call back into applications.
 */

void hall_setcallback(FAR struct hall_lowerhalf_s *lower,
                         CODE xcpt_t callback, FAR void *arg)
{

	  irqstate_t flags = enter_critical_section();

	  /* Save the new callback */
	  if (callback != NULL ){
	      /* Yes.. Defer the hander attachment to the lower half driver */
		  lower->handle = callback;
		  stm32_gpiosetevent(lower->pin, lower->risingedge, lower->fallingedge, lower->event, lower->handle, NULL);

	  }

	  leave_critical_section(flags);
}




/****************************************************************************
 * Name: stm32_hall_setup
 *
 * Description:
 *   stm32_hall_setup() must be called to initialize hall resources.
 *
 ****************************************************************************/

int stm32_hall_setup()
{
	void *ret = NULL;

#ifdef CONFIG_HALL_DEV1
	ret = hall_register("/dev/hall1",&g_hall1_lower);
	if(ret == NULL){
		syslog(LOG_ERR, "[HALL]: Failed to register hall1 \n");
		return -EINVAL;
	}
#endif

#ifdef CONFIG_HALL_DEV2
	ret = hall_register("/dev/hall2",&g_hall2_lower);
	if(ret == NULL){
		syslog(LOG_ERR, "[HALL]: Failed to register hall2 \n");
		return -EINVAL;
	}
#endif

#ifdef CONFIG_HALL_DEV3
	ret = hall_register("/dev/hall3",&g_hall3_lower);
	if(ret == NULL){
		syslog(LOG_ERR, "[HALL]: Failed to register hall3 \n");
		return -EINVAL;
	}
#endif

#ifdef CONFIG_HALL_DEV4
	ret = hall_register("/dev/hall4",&g_hall4_lower);
	if(ret == NULL){
		syslog(LOG_ERR, "[HALL]: Failed to register hall4 \n");
		return -EINVAL;
	}
#endif

#ifdef CONFIG_HALL_DEV5
	ret = hall_register("/dev/hall5",&g_hall5_lower);
	if(ret == NULL){
		syslog(LOG_ERR, "[HALL]: Failed to register hall5 \n");
		return -EINVAL;
	}
#endif

#ifdef CONFIG_HALL_DEV6
	ret = hall_register("/dev/hall6",&g_hall6_lower);
	if(ret == NULL){
		syslog(LOG_ERR, "[HALL]: Failed to register hall6 \n");
		return -EINVAL;
	}
#endif

	return OK;
}

#endif /* CONFIG_HALLS */

