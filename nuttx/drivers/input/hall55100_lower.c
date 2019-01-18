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

//#ifdef CONFIG_HALLS
/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <stdint.h>
#include <errno.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include "stm32_gpio.h"
#include "hall55100.h"

//#ifdef CONFIG_ARCH_HALLS


int hall_start(FAR struct hall_lowerhalf_s *lower);
int hall_stop(FAR struct hall_lowerhalf_s *lower);
int hall_ioctl(FAR struct hall_lowerhalf_s *lower, int cmd,
                  unsigned long arg);
void hall_setcallback(FAR struct hall_lowerhalf_s *lower,
                         CODE tccb_t callback, FAR void *arg);
int hall1_handle(int irq, FAR void *context, FAR void *arg);
int hall2_handle(int irq, FAR void *context, FAR void *arg);
int hall3_handle(int irq, FAR void *context, FAR void *arg);
int hall4_handle(int irq, FAR void *context, FAR void *arg);
int hall5_handle(int irq, FAR void *context, FAR void *arg);
int hall6_handle(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct hall_ops_s g_hall_ops =
{
	.start 		 = hall_start,
	.stop 		 = hall_stop,
	.ioctl 		 = hall_stop,
	.setcallback = hall_setcallback
};


/*
 * Pin configuration for each hall.
 */

static struct hall_lowerhalf_s g_hall1_lower =
{
	.ops 		 = &g_hall_ops
	.handle 	 = hall1_handle,
	.pin 		 = GPIO_HW_RXIN1,
	.risingedge  = true,
	.fallingedge = true,
	.event       = true
};

static struct hall_lowerhalf_s g_hall2_lower =
{
	.ops 	= &g_hall_ops
	.handle = hall2_handle,
	.pin 	= GPIO_HW_RXIN2,
	.risingedge  = true,
	.fallingedge = true,
	.event       = true
};

static struct hall_lowerhalf_s g_hall3_lower =
{
	.ops 	= &g_hall_ops
	.handle = hall3_handle,
	.pin 	= GPIO_HW_RXIN3,
	.risingedge  = true,
	.fallingedge = true,
	.event       = true
};

static struct hall_lowerhalf_s g_hall4_lower =
{
	.ops 	= &g_hall_ops
	.handle = hall4_handle,
	.pin 	= GPIO_HW_RXIN4,
	.risingedge  = true,
	.fallingedge = true,
	.event       = true
};

static struct hall_lowerhalf_s g_hall5_lower =
{
	.ops 	= &g_hall_ops
	.handle = hall5_handle,
	.pin 	= GPIO_HW_RXIN5,
	.risingedge  = true,
	.fallingedge = true,
	.event       = true
};

static struct hall_lowerhalf_s g_hall6_lower =
{
	.ops 	= &g_hall_ops
	.handle = hall6_handle,
	.pin 	= GPIO_HW_RXIN6,
	.risingedge  = true,
	.fallingedge = true,
	.event       = true
};





/****************************************************************************
 * Public Functions
 ****************************************************************************/

int hall1_handle(int irq, FAR void *context, FAR void *arg)
{
	return 1;
}

int hall2_handle(int irq, FAR void *context, FAR void *arg)
{
	return 1;
}

int hall3_handle(int irq, FAR void *context, FAR void *arg)
{
	return 1;
}

int hall4_handle(int irq, FAR void *context, FAR void *arg)
{
	return 1;
}

int hall5_handle(int irq, FAR void *context, FAR void *arg)
{
	return 1;
}

int hall6_handle(int irq, FAR void *context, FAR void *arg)
{
	return 1;
}


/* Start the timer, resetting the time to the current timeout */

int hall_start(FAR struct hall_lowerhalf_s *lower)
{
	int ret = -1;
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
	;
}

/* Call the NuttX INTERNAL timeout callback on timeout.
 * NOTE:  Providing callback==NULL disable.
 * NOT to call back into applications.
 */

void hall_setcallback(FAR struct hall_lowerhalf_s *lower,
                         CODE xcpt_t callback, FAR void *arg)
{
	  FAR struct hall_lowerhalf_s *priv = (FAR struct stm32_lowerhalf_s *)lower;

	  irqstate_t flags = enter_critical_section();

	  /* Save the new callback */
	  if (callback != NULL ){
	      /* Yes.. Defer the hander attachment to the lower half driver */
		  priv->handle = callback;
	  }

	  leave_critical_section(flags);
}



































/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hall1_handle
 *
 * Description:
 *   board_hall_initialize() must be called to initialize hall resources.  After
 *   that, board_halls() may be called to collect the current state of all
 *   halls or board_hall_irq() may be called to register hall interrupt
 *   handlers.
 *
 ****************************************************************************/





/****************************************************************************
 * Name: board_halls
 ****************************************************************************/

uint32_t board_halls(void)
{
  uint32_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < NUM_HALLS; i++)
    {
       /* A LOW value means that the key is pressed.
        */

       bool released = stm32_gpioread(g_halls[i]);

       /* Accumulate the set of depressed (not released) keys */

       if (!released)
         {
            ret |= (1 << i);
         }
    }

  return ret;
}

/************************************************************************************
 * hall support.
 *
 * Description:
 *   board_hall_initialize() must be called to initialize hall resources.  After
 *   that, board_halls() may be called to collect the current state of all
 *   halls or board_hall_irq() may be called to register hall interrupt
 *   handlers.
 *
 *   After board_hall_initialize() has been called, board_halls() may be called to
 *   collect the state of all halls.  board_halls() returns an 32-bit bit set
 *   with each bit associated with a hall.  See the hall_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   board_hall_irq() may be called to register an interrupt handler that will
 *   be called when a hall is depressed or released.  The ID value is a
 *   hall enumeration value that uniquely identifies a hall resource. See the
 *   hall_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ************************************************************************************/

//#ifdef CONFIG_ARCH_IRQHALLS
int board_hall_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret = -EINVAL;

  /* The following should be atomic */

  if (id >= MIN_IRQHALL && id <= MAX_IRQHALL)
    {
      ret = stm32_gpiosetevent(g_halls[id], true, true, true, irqhandler, arg);
    }

  return ret;
}

/****************************************************************************
 * Name: board_hall_initialize
 *
 * Description:
 *   board_hall_initialize() must be called to initialize hall resources.  After
 *   that, board_halls() may be called to collect the current state of all
 *   halls or board_hall_irq() may be called to register hall interrupt
 *   handlers.
 *
 ****************************************************************************/

void hall_set_up(void)
{
  int i;

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for all pins.
   */

  for (i = 0; i < NUM_HALLS; i++)
    {
      stm32_configgpio(g_halls[i]);
      board_hall_irq(i, g_hall_handle[i], NULL);
    }

}


//#endif /* CONFIG_HALLS */

