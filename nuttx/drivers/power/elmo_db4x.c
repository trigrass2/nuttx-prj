/****************************************************************************
 * drivers/power/elmo_db4x.c
 * Upper-half, character driver for elmo_db4x control
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>

#include <nuttx/power/motor.h>
#include <nuttx/power/elmo_db4x.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MOTOR_STATE_BIT_MOVE		0X01
#define MOTOR_STATE_BIT_ONLINE		0X02
#define MOTOR_STATE_BIT_ERR			0X04

#define MOTOR_DATA_TYPE_READONLY	1

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* elmodb4x driver methods */

static int  elmo_db4x_setup(FAR struct motor_dev_s *dev);
static int  elmo_db4x_shutdown(FAR struct motor_dev_s *dev);
static int  elmo_db4x_stop(FAR struct motor_dev_s *dev);
static int  elmo_db4x_start(FAR struct motor_dev_s *dev);
static int  elmo_db4x_params_set(FAR struct motor_dev_s *dev,FAR struct motor_params_s *param);
static int  elmo_db4x_params_get(FAR struct motor_dev_s *dev,FAR struct motor_params_s *param);
static int  elmo_db4x_mode_set(FAR struct motor_dev_s *dev, uint8_t mode);
static int  elmo_db4x_limits_set(FAR struct motor_dev_s *dev,FAR struct motor_limits_s *limits);
static int  elmo_db4x_fault_set(FAR struct motor_dev_s *dev, uint8_t fault);
static int  elmo_db4x_state_get(FAR struct motor_dev_s *dev,FAR struct motor_state_s *state);
static int  elmo_db4x_state_set(FAR struct motor_dev_s *dev,FAR struct motor_state_s *state);
static int  elmo_db4x_fault_get(FAR struct motor_dev_s *dev, FAR uint8_t *fault);
static int  elmo_db4x_fault_clean(FAR struct motor_dev_s *dev, uint8_t fault);
static int  elmo_db4x_ioctl(FAR struct motor_dev_s *dev, int cmd,unsigned long arg);


/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half elmo driver methods used by the upper half driver */

static struct motor_ops_s g_motorops =
{
	/* Configure motor */
	.setup		= elmo_db4x_setup,		/* Setup motor */
	.shutdown	= elmo_db4x_shutdown,	/* shutdown motor */
	.stop		= elmo_db4x_stop,		/* Stop motor */
	.start		= elmo_db4x_start,		/* Start motor */
	.params_set	= elmo_db4x_params_set,	/* Set motor parameters */
	.params_get	= elmo_db4x_params_get,	/* get motor parameters */
	.mode_set	= elmo_db4x_mode_set,	/* Set motor operation mode */
	.limits_set	= elmo_db4x_limits_set, /* Set motor limts */
	.fault_set	= elmo_db4x_fault_set,	/* Set motor fault */
	.state_get	= elmo_db4x_state_get,	/* Get motor state  */
	.state_set	= elmo_db4x_state_set,	/* Set motor state  */
	.fault_get	= elmo_db4x_fault_get,  /* Get current fault state */
	.fault_clean= elmo_db4x_fault_clean,/* Clean fault state */
	.ioctl		= elmo_db4x_ioctl		/* may support platform-specific ioctl commands */
};

static struct elmo_dops_s g_elmo_dops =
{
	.set_params_to_drv	 = Motor_ELMO_Parameter_Set,	/* Set params to elmo driver */
	.init_drv 			 = Elmo_Init,					/* Initialize elmo driver */
	.set_position_to_drv = Motor_Position_Move,			/* Set elmo driver position */
	.send_sync_to_drv	 = Motor_ELMO_State_Get,		/* Send elmo driver sync */
};


//#if defined(CONFIG_ELMO_MOTOR_1)

static struct motor_s g_elmo_motor_1_priv =
{
	.opmode		= MOTOR_OPMODE_SPEED,
	.opflags 	= MOTOR_OPMODE_SPEED,
	.id			= 127,
	/* limits */
	.limits =
	{
		#ifdef CONFIG_MOTOR_HAVE_POSITION
			.position = 1080,                   /* Maximum motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
			.speed = 720,                       /* Maximum motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
			.torque = 0,                      /* Maximum motor torque (rotary motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
			.force =  0,                      /* Maximum motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
			.acceleration = 1000.0,            /* Maximum motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
			.deceleration = 1000.0,            /* Maximum motor decelaration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			.v_in = 0.0,                        /* Maximum input voltage */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			.i_in = 0.0,                        /* Maximum input current */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			.p_in = 0.0                        /* Maximum input power */
		#endif
	},
	/* parameter */
	.param =
	{
		  .lock = false,/* Lock this structure. Set this bit  if there is no need to change motor parameter during run-time.*/
		#ifdef CONFIG_MOTOR_HAVE_DIRECTION
		 .direction = true,                   /* Motor movement direction. We do not support negative values for parameters,
											  * so this flag can be used to allow movement in the positive and negative direction in a given coordinate system.*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_POSITION
		  .position = 0.0,                    /* Motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
		  .speed = 0.0,                       /* Motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
		  .torque = 0.0,                      /* Motor torque (rotary motor)*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
		  .force = 0.0;                       /* Motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
		  .acceleration = 1000.0,                /* Motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
		  .deceleartion = 10000.0                 /* Motor deceleration Unit:degree/s/s */
		#endif
	},
	/* state */
	.state =
	{
	  .state = MOTOR_STATE_INIT,     /* Motor state  */
	  .fault = 0,     				 /* Motor faults state */
	  .fb =         				 /* Feedback from motor */
	  {
			#ifdef CONFIG_MOTOR_HAVE_POSITION
			  .position = 0.0,               /* Current motor position */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_SPEED
			  .speed = 0.0,                  /* Current motor speed */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_TORQUE
			  .torque = 0.0,                 /* Current motor torque (rotary motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_FORCE
			  .force = 0.0,                  /* Current motor force (linear motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			  .v_in = 0.0,                   /* Current input voltage */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			  .i_in = 0.0,                   /* Current input current */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			  .p_in = 0.0,                   /* Current input power */
			#endif
	  }
	},
	.priv = NULL
};


static struct motor_s g_elmo_motor_2_priv =
{
	.opmode		= MOTOR_OPMODE_SPEED,
	.opflags 	= MOTOR_OPMODE_SPEED,
	.id			= 126,
	/* limits */
	.limits =
	{
		#ifdef CONFIG_MOTOR_HAVE_POSITION
			.position = 1080,                   /* Maximum motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
			.speed = 720,                       /* Maximum motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
			.torque = 0,                      /* Maximum motor torque (rotary motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
			.force =  0,                      /* Maximum motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
			.acceleration = 1000.0,            /* Maximum motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
			.deceleration = 1000.0,            /* Maximum motor decelaration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			.v_in = 0.0,                        /* Maximum input voltage */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			.i_in = 0.0,                        /* Maximum input current */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			.p_in = 0.0                        /* Maximum input power */
		#endif
	},
	/* parameter */
	.param =
	{
		  .lock = false,/* Lock this structure. Set this bit  if there is no need to change motor parameter during run-time.*/
		#ifdef CONFIG_MOTOR_HAVE_DIRECTION
		 .direction = true,                   /* Motor movement direction. We do not support negative values for parameters,
											  * so this flag can be used to allow movement in the positive and negative direction in a given coordinate system.*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_POSITION
		  .position = 0.0,                    /* Motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
		  .speed = 0.0,                       /* Motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
		  .torque = 0.0,                      /* Motor torque (rotary motor)*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
		  .force = 0.0;                       /* Motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
		  .acceleration = 1000.0,                /* Motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
		  .deceleartion = 10000.0                 /* Motor deceleration Unit:degree/s/s */
		#endif
	},
	/* state */
	.state =
	{
	  .state = MOTOR_STATE_INIT,     /* Motor state  */
	  .fault = 0,     				 /* Motor faults state */
	  .fb =         				 /* Feedback from motor */
	  {
			#ifdef CONFIG_MOTOR_HAVE_POSITION
			  .position = 0.0,               /* Current motor position */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_SPEED
			  .speed = 0.0,                  /* Current motor speed */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_TORQUE
			  .torque = 0.0,                 /* Current motor torque (rotary motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_FORCE
			  .force = 0.0,                  /* Current motor force (linear motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			  .v_in = 0.0,                   /* Current input voltage */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			  .i_in = 0.0,                   /* Current input current */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			  .p_in = 0.0,                   /* Current input power */
			#endif
	  }
	},
	.priv = NULL
};


static struct motor_s g_elmo_motor_3_priv =
{
	.opmode		= MOTOR_OPMODE_SPEED,
	.opflags 	= MOTOR_OPMODE_SPEED,
	.id			= 125,
	/* limits */
	.limits =
	{
		#ifdef CONFIG_MOTOR_HAVE_POSITION
			.position = 1080,                   /* Maximum motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
			.speed = 720,                       /* Maximum motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
			.torque = 0,                      /* Maximum motor torque (rotary motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
			.force =  0,                      /* Maximum motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
			.acceleration = 1000.0,            /* Maximum motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
			.deceleration = 1000.0,            /* Maximum motor decelaration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			.v_in = 0.0,                        /* Maximum input voltage */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			.i_in = 0.0,                        /* Maximum input current */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			.p_in = 0.0                        /* Maximum input power */
		#endif
	},
	/* parameter */
	.param =
	{
		  .lock = false,/* Lock this structure. Set this bit  if there is no need to change motor parameter during run-time.*/
		#ifdef CONFIG_MOTOR_HAVE_DIRECTION
		 .direction = true,                   /* Motor movement direction. We do not support negative values for parameters,
											  * so this flag can be used to allow movement in the positive and negative direction in a given coordinate system.*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_POSITION
		  .position = 0.0,                    /* Motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
		  .speed = 0.0,                       /* Motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
		  .torque = 0.0,                      /* Motor torque (rotary motor)*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
		  .force = 0.0;                       /* Motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
		  .acceleration = 1000.0,                /* Motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
		  .deceleartion = 10000.0                 /* Motor deceleration Unit:degree/s/s */
		#endif
	},
	/* state */
	.state =
	{
	  .state = MOTOR_STATE_INIT,     /* Motor state  */
	  .fault = 0,     				 /* Motor faults state */
	  .fb =         				 /* Feedback from motor */
	  {
			#ifdef CONFIG_MOTOR_HAVE_POSITION
			  .position = 0.0,               /* Current motor position */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_SPEED
			  .speed = 0.0,                  /* Current motor speed */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_TORQUE
			  .torque = 0.0,                 /* Current motor torque (rotary motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_FORCE
			  .force = 0.0,                  /* Current motor force (linear motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			  .v_in = 0.0,                   /* Current input voltage */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			  .i_in = 0.0,                   /* Current input current */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			  .p_in = 0.0,                   /* Current input power */
			#endif
	  }
	},
	.priv = NULL
};


static struct motor_s g_elmo_motor_4_priv =
{
	.opmode		= MOTOR_OPMODE_SPEED,
	.opflags 	= MOTOR_OPMODE_SPEED,
	.id			= 124,
	/* limits */
	.limits =
	{
		#ifdef CONFIG_MOTOR_HAVE_POSITION
			.position = 1080,                   /* Maximum motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
			.speed = 720,                       /* Maximum motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
			.torque = 0,                      /* Maximum motor torque (rotary motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
			.force =  0,                      /* Maximum motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
			.acceleration = 1000.0,            /* Maximum motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
			.deceleration = 1000.0,            /* Maximum motor decelaration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			.v_in = 0.0,                        /* Maximum input voltage */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			.i_in = 0.0,                        /* Maximum input current */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			.p_in = 0.0                        /* Maximum input power */
		#endif
	},
	/* parameter */
	.param =
	{
		  .lock = false,/* Lock this structure. Set this bit  if there is no need to change motor parameter during run-time.*/
		#ifdef CONFIG_MOTOR_HAVE_DIRECTION
		 .direction = true,                   /* Motor movement direction. We do not support negative values for parameters,
											  * so this flag can be used to allow movement in the positive and negative direction in a given coordinate system.*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_POSITION
		  .position = 0.0,                    /* Motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
		  .speed = 0.0,                       /* Motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
		  .torque = 0.0,                      /* Motor torque (rotary motor)*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
		  .force = 0.0;                       /* Motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
		  .acceleration = 1000.0,                /* Motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
		  .deceleartion = 10000.0                 /* Motor deceleration Unit:degree/s/s */
		#endif
	},
	/* state */
	.state =
	{
	  .state = MOTOR_STATE_INIT,     /* Motor state  */
	  .fault = 0,     				 /* Motor faults state */
	  .fb =         				 /* Feedback from motor */
	  {
			#ifdef CONFIG_MOTOR_HAVE_POSITION
			  .position = 0.0,               /* Current motor position */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_SPEED
			  .speed = 0.0,                  /* Current motor speed */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_TORQUE
			  .torque = 0.0,                 /* Current motor torque (rotary motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_FORCE
			  .force = 0.0,                  /* Current motor force (linear motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			  .v_in = 0.0,                   /* Current input voltage */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			  .i_in = 0.0,                   /* Current input current */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			  .p_in = 0.0,                   /* Current input power */
			#endif
	  }
	},
	.priv = NULL
};


static struct motor_s g_elmo_motor_5_priv =
{
	.opmode		= MOTOR_OPMODE_SPEED,
	.opflags 	= MOTOR_OPMODE_SPEED,
	.id			= 123,
	/* limits */
	.limits =
	{
		#ifdef CONFIG_MOTOR_HAVE_POSITION
			.position = 1080,                   /* Maximum motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
			.speed = 720,                       /* Maximum motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
			.torque = 0,                      /* Maximum motor torque (rotary motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
			.force =  0,                      /* Maximum motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
			.acceleration = 1000.0,            /* Maximum motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
			.deceleration = 1000.0,            /* Maximum motor decelaration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			.v_in = 0.0,                        /* Maximum input voltage */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			.i_in = 0.0,                        /* Maximum input current */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			.p_in = 0.0                        /* Maximum input power */
		#endif
	},
	/* parameter */
	.param =
	{
		  .lock = false,/* Lock this structure. Set this bit  if there is no need to change motor parameter during run-time.*/
		#ifdef CONFIG_MOTOR_HAVE_DIRECTION
		 .direction = true,                   /* Motor movement direction. We do not support negative values for parameters,
											  * so this flag can be used to allow movement in the positive and negative direction in a given coordinate system.*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_POSITION
		  .position = 0.0,                    /* Motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
		  .speed = 0.0,                       /* Motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
		  .torque = 0.0,                      /* Motor torque (rotary motor)*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
		  .force = 0.0;                       /* Motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
		  .acceleration = 1000.0,                /* Motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
		  .deceleartion = 10000.0                 /* Motor deceleration Unit:degree/s/s */
		#endif
	},
	/* state */
	.state =
	{
	  .state = MOTOR_STATE_INIT,     /* Motor state  */
	  .fault = 0,     				 /* Motor faults state */
	  .fb =         				 /* Feedback from motor */
	  {
			#ifdef CONFIG_MOTOR_HAVE_POSITION
			  .position = 0.0,               /* Current motor position */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_SPEED
			  .speed = 0.0,                  /* Current motor speed */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_TORQUE
			  .torque = 0.0,                 /* Current motor torque (rotary motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_FORCE
			  .force = 0.0,                  /* Current motor force (linear motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			  .v_in = 0.0,                   /* Current input voltage */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			  .i_in = 0.0,                   /* Current input current */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			  .p_in = 0.0,                   /* Current input power */
			#endif
	  }
	},
	.priv = NULL
};


static struct motor_s g_elmo_motor_6_priv =
{
	.opmode		= MOTOR_OPMODE_SPEED,
	.opflags 	= MOTOR_OPMODE_SPEED,
	.id			= 122,
	/* limits */
	.limits =
	{
		#ifdef CONFIG_MOTOR_HAVE_POSITION
			.position = 1080,                   /* Maximum motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
			.speed = 720,                       /* Maximum motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
			.torque = 0,                      /* Maximum motor torque (rotary motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
			.force =  0,                      /* Maximum motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
			.acceleration = 1000.0,            /* Maximum motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
			.deceleration = 1000.0,            /* Maximum motor decelaration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			.v_in = 0.0,                        /* Maximum input voltage */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			.i_in = 0.0,                        /* Maximum input current */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			.p_in = 0.0                        /* Maximum input power */
		#endif
	},
	/* parameter */
	.param =
	{
		  .lock = false,/* Lock this structure. Set this bit  if there is no need to change motor parameter during run-time.*/
		#ifdef CONFIG_MOTOR_HAVE_DIRECTION
		 .direction = true,                   /* Motor movement direction. We do not support negative values for parameters,
											  * so this flag can be used to allow movement in the positive and negative direction in a given coordinate system.*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_POSITION
		  .position = 0.0,                    /* Motor position */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_SPEED
		  .speed = 0.0,                       /* Motor speed */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_TORQUE
		  .torque = 0.0,                      /* Motor torque (rotary motor)*/
		#endif
		#ifdef CONFIG_MOTOR_HAVE_FORCE
		  .force = 0.0;                       /* Motor force (linear motor) */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
		  .acceleration = 1000.0,                /* Motor acceleration Unit:degree/s/s */
		#endif
		#ifdef CONFIG_MOTOR_HAVE_DECELERATION
		  .deceleartion = 10000.0                 /* Motor deceleration Unit:degree/s/s */
		#endif
	},
	/* state */
	.state =
	{
	  .state = MOTOR_STATE_INIT,     /* Motor state  */
	  .fault = 0,     				 /* Motor faults state */
	  .fb =         				 /* Feedback from motor */
	  {
			#ifdef CONFIG_MOTOR_HAVE_POSITION
			  .position = 0.0,               /* Current motor position */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_SPEED
			  .speed = 0.0,                  /* Current motor speed */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_TORQUE
			  .torque = 0.0,                 /* Current motor torque (rotary motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_FORCE
			  .force = 0.0,                  /* Current motor force (linear motor) */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
			  .v_in = 0.0,                   /* Current input voltage */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
			  .i_in = 0.0,                   /* Current input current */
			#endif
			#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
			  .p_in = 0.0,                   /* Current input power */
			#endif
	  }
	},
	.priv = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: elmo_db4x_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int elmo_db4x_setup(FAR struct motor_dev_s *dev)
{
  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;
  FAR struct elmo_dops_s *motor_lower = (FAR struct elmo_dops_s *)motor_dev->lower;

  /* Configure the elmodb4x driver motor over canopen */

  /* 1、Set motor operation mode */
  motor_priv->opmode = MOTOR_OPMODE_POSITION_ABSOLUTE;

  /* 2、Set motor parameters */
  motor_priv->param.lock = false;
#ifdef CONFIG_MOTOR_HAVE_DIRECTION
  motor_priv->param.direction = false; //false:CW  true:CCW
#endif
#ifdef CONFIG_MOTOR_HAVE_POSITION
  motor_priv->param.position = 0.0;
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
  motor_priv->param.speed = 0.0;
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
  motor_priv->param.torque = 0.0;                      /* Motor torque (rotary motor)*/
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
  motor_priv->param.force = 0.0;                       /* Motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
  motor_priv->param.acceleration = 720.0;
#endif
#ifdef CONFIG_MOTOR_HAVE_DECELERATION
  motor_priv->param.deceleartion = 720.0;
#endif

  /* 3、Set motor absolute limits */
  //motor_priv->limits.lock = false;
  motor_priv->limits.lock = true;
#ifdef CONFIG_MOTOR_HAVE_POSITION
  motor_priv->limits.position = 1080.0;
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
  motor_priv->limits.speed = 720.0;
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
  motor_priv->limits.torque = 1.0;                      /* Maximum motor torque (rotary motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
  motor_priv->limits.force = 1.0;                       /* Maximum motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
  motor_priv->limits.acceleration = 1.0;                /* Maximum motor acceleration */
#endif
#ifdef CONFIG_MOTOR_HAVE_DECELERATION
  motor_priv->limits.deceleration = 1.0;                /* Maximum motor decelaration */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
  motor_priv->limits.v_in = 0.0;                        /* Maximum input voltage */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
  motor_priv->limits.i_in = 0.0;                        /* Maximum input current */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
  motor_priv->limits.p_in = 0.0;                        /* Maximum input power */
#endif

  /* 4、Clean motor state */
  motor_priv->state.state = 0;
  motor_priv->state.fault = 0;
#ifdef CONFIG_MOTOR_HAVE_POSITION
  motor_priv->state.fb.position = 0.0;               /* Current motor position */
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
  motor_priv->state.fb.speed = 0.0;                  /* Current motor speed */
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
  motor_priv->state.fb.torque = 0.0;                 /* Current motor torque (rotary motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
  motor_priv->state.fb.force = 0.0;                  /* Current motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
  motor_priv->state.fb.v_in = 0.0;                   /* Current input voltage */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
  motor_priv->state.fb.i_in = 0.0;                   /* Current input current */
#endif
#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
  motor_priv->state.fb.p_in = 0.0;                   /* Current input power */
#endif

  /* 5、Clean motor operation flags */
  motor_priv->opflags = MOTOR_STATE_INIT;

  /* 6、Set to elmo */
  motor_lower->init_drv(motor_priv->id,motor_priv->priv);

  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_shutdown
 *
 * Description:
 *   This method is called when the driver is shutdown.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int elmo_db4x_shutdown(FAR struct motor_dev_s *dev)
{
  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

  /* Shutdown the elmodb4x driver motor over canopen */
  motor_priv->opmode = MOTOR_OPMODE_SHUTDOWN;
  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_stop
 *
 * Description:
 *   This method is called when the driver need stop.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int elmo_db4x_stop(FAR struct motor_dev_s *dev)
{
  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

  /* Stop the elmodb4x driver motor over canopen */
  motor_priv->opmode = MOTOR_OPMODE_STOP;

  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_start
 *
 * Description:
 *   This method is called when the driver need start.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int elmo_db4x_start(FAR struct motor_dev_s *dev)
{
  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;
  /* Stop the elmodb4x driver motor over canopen */

  motor_priv->opmode = MOTOR_OPMODE_START;
  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_params_set
 *
 * Description:
 *   This method is called when the driver need set params.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int elmo_db4x_params_set(FAR struct motor_dev_s *dev,FAR struct motor_params_s *param)
{
  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

  /* set the elmodb4x driver motor parameters */
  motor_priv->param.lock = param->lock;
#ifdef CONFIG_MOTOR_HAVE_DIRECTION
  motor_priv->param.direction = param->direction; //false:CW  true:CCW
#endif
#ifdef CONFIG_MOTOR_HAVE_POSITION
  motor_priv->param.position = param->position;
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
  motor_priv->param.speed = param->speed;
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
  motor_priv->param.torque = param->torque;                      /* Motor torque (rotary motor)*/
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
  motor_priv->param.force = param->rorce;                       /* Motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
  motor_priv->param.acceleration = param->acceleration;
#endif
#ifdef CONFIG_MOTOR_HAVE_DECELERATION
  motor_priv->param.deceleartion = param->deceleartion;
#endif

  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_params_set
 *
 * Description:
 *   This method is called when the driver need set params.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/

static int elmo_db4x_params_get(FAR struct motor_dev_s *dev,FAR struct motor_params_s *param)
{
  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

  /* set the elmodb4x driver motor parameters */
  param->lock = motor_priv->param.lock;
#ifdef CONFIG_MOTOR_HAVE_DIRECTION
  param->deceleartion = motor_priv->param.direction; //false:CW  true:CCW
#endif
#ifdef CONFIG_MOTOR_HAVE_POSITION
   param->position = motor_priv->param.position;
#endif
#ifdef CONFIG_MOTOR_HAVE_SPEED
  param->speed = motor_priv->param.speed;
#endif
#ifdef CONFIG_MOTOR_HAVE_TORQUE
  param->torque = motor_priv->param.torque;                      /* Motor torque (rotary motor)*/
#endif
#ifdef CONFIG_MOTOR_HAVE_FORCE
  param->rorce = motor_priv->param.force;                       /* Motor force (linear motor) */
#endif
#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
  param->acceleration = motor_priv->param.acceleration;
#endif
#ifdef CONFIG_MOTOR_HAVE_DECELERATION
  param->deceleartion = motor_priv->param.deceleartion;
#endif

  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_mode_set
 *
 * Description:
 *   This method is called when the driver need set mode.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static int  elmo_db4x_mode_set(FAR struct motor_dev_s *dev, uint8_t mode)
{
	  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
	  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

	  /* set the elmodb4x driver motor mode over canopen */
	  motor_priv->opmode = mode;

	  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_limits_set
 *
 * Description:
 *   This method is called when the driver need set limits.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static int  elmo_db4x_limits_set(FAR struct motor_dev_s *dev,FAR struct motor_limits_s *limits)
{
	  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
	  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

	  /* set the elmodb4x driver motor limits over canopen */
	  /* 3、Set motor absolute limits */
	  motor_priv->limits.lock = limits->lock;
	#ifdef CONFIG_MOTOR_HAVE_POSITION
	  motor_priv->limits.position = limits->position;
	#endif
	#ifdef CONFIG_MOTOR_HAVE_SPEED
	  motor_priv->limits.speed = limits->speed;
	#endif
	#ifdef CONFIG_MOTOR_HAVE_TORQUE
	  motor_priv->limits.torque = limits->torque;                      /* Maximum motor torque (rotary motor) */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_FORCE
	  motor_priv->limits.force = limits->force;                       /* Maximum motor force (linear motor) */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
	  motor_priv->limits.acceleration = limits->acceleration;                /* Maximum motor acceleration */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_DECELERATION
	  motor_priv->limits.deceleration = limits->deceleration;                /* Maximum motor decelaration */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
	  motor_priv->limits.v_in = limits->v_in;                        /* Maximum input voltage */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
	  motor_priv->limits.i_in = limits->i_in;                        /* Maximum input current */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
	  motor_priv->limits.p_in = limits->p_in;                        /* Maximum input power */
	#endif

	  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_fault_set
 *
 * Description:
 *   This method is called when the driver need set fault.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static int  elmo_db4x_fault_set(FAR struct motor_dev_s *dev, uint8_t fault)
{
	  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
	  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

	  /* set the elmodb4x driver motor fault over canopen */
	  motor_priv->state.fault |= fault;

	  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_fault_get
 *
 * Description:
 *   This method is called when the driver need get fault.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static int  elmo_db4x_fault_get(FAR struct motor_dev_s *dev, FAR uint8_t *fault)
{
	  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
	  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

	  /* get the elmodb4x driver motor fault over canopen */
	  *fault = motor_priv->state.fault;
	  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_fault_clean
 *
 * Description:
 *   This method is called when the driver need clean fault.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static int  elmo_db4x_fault_clean(FAR struct motor_dev_s *dev, uint8_t fault)
{
	  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
	  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

	  /* clean the elmodb4x driver motor fault over canopen */
	  motor_priv->state.fault &= ~fault;
	  return OK;
}


/****************************************************************************
 * Name: elmo_db4x_state_get
 *
 * Description:
 *   This method is called when the driver need get state.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static int  elmo_db4x_state_get(FAR struct motor_dev_s *dev,FAR struct motor_state_s *state)
{
	  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
	  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

	  /* get the elmodb4x driver motor state over canopen */
	  state->state = motor_priv->state.state;
	  state->fault = motor_priv->state.fault;
	#ifdef CONFIG_MOTOR_HAVE_POSITION
	  state->fb.position = motor_priv->state.fb.position;               /* Current motor position */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_SPEED
	  state->fb.speed = motor_priv->state.fb.speed;                     /* Current motor speed */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_TORQUE
	  state->fb.torque = motor_priv->state.fb.torque;                   /* Current motor torque (rotary motor) */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_FORCE
	  state->fb.force = motor_priv->state.fb.force;                     /* Current motor force (linear motor) */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
	  state->fb.v_in = motor_priv->state.fb.v_in;                      /* Current input voltage */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
	  state->fb.i_in = motor_priv->state.fb.i_in;                      /* Current input current */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
	  state->fb.p_in = motor_priv->state.fb.p_in;                      /* Current input power */
	#endif

	  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_state_set
 *
 * Description:
 *   This method is called when the driver need set state.
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static int  elmo_db4x_state_set(FAR struct motor_dev_s *dev,FAR struct motor_state_s *state)
{
	  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
	  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;

	  /* get the elmodb4x driver motor state over canopen */
	  motor_priv->state.state = state->state;
	  motor_priv->state.fault = state->fault;
	#ifdef CONFIG_MOTOR_HAVE_POSITION
	  motor_priv->state.fb.position = state->fb.position;               /* Current motor position */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_SPEED
	  motor_priv->state.fb.speed = state->fb.speed;                     /* Current motor speed */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_TORQUE
	  motor_priv->state.fb.torque = state->fb.torque;                   /* Current motor torque (rotary motor) */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_FORCE
	  motor_priv->state.fb.force = state->fb.force;                     /* Current motor force (linear motor) */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_VOLTAGE
	  motor_priv->state.fb.v_in = state->fb.v_in;                      /* Current input voltage */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_CURRENT
	  motor_priv->state.fb.i_in = state->fb.i_in;                      /* Current input current */
	#endif
	#ifdef CONFIG_MOTOR_HAVE_INPUT_POWER
	  motor_priv->state.fb.p_in = state->fb.p_in;                      /* Current input power */
	#endif

	  return OK;
}

/****************************************************************************
 * Name: elmo_db4x_ioctl
 *
 * Description:
 *
 * Input Parameters:
 *   dev - A reference to the driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *
 ****************************************************************************/
static int  elmo_db4x_ioctl(FAR struct motor_dev_s *dev, int cmd,unsigned long arg)
{
	  FAR struct motor_dev_s *motor_dev = (FAR struct motor_dev_s *)dev;
	  FAR struct motor_s *motor_priv = (FAR struct motor_s *)motor_dev->priv;
	  FAR struct elmo_dops_s *motor_lower = (FAR struct elmo_dops_s *)motor_dev->lower;

	  switch(cmd)
	  {
	  	  case PWRIOC_RUN_PARAMS:
	  	  {
#ifdef CONFIG_MOTOR_HAVE_SPEED
			  /* 1:velocity */
			  motor_lower->set_params_to_drv(motor_priv->id, VELOCITY,&(motor_priv->param.speed));
#endif

#ifdef CONFIG_MOTOR_HAVE_ACCELERATION
			  /* 2:acceleration */
			  motor_lower->set_params_to_drv(motor_priv->id, ACCELERATE,&(motor_priv->param.acceleration));
#endif

#ifdef CONFIG_MOTOR_HAVE_DECELERATION
			  /* 3:deceleration */
			  motor_lower->set_params_to_drv(motor_priv->id, DECELERATE,&(motor_priv->param.deceleartion));
#endif
			  motor_lower->set_params_to_drv(motor_priv->id, MODE_TYPE,&(motor_priv->opmode));
	  	  }
	  		  break;

	  	  case PWRIOC_RUN_POSITIN:
	  	  {
	  		  if(motor_priv->opmode == MOTOR_OPMODE_POSITION_ABSOLUTE){
	  			motor_lower->set_position_to_drv(motor_priv->param.position,motor_priv->id,ELMO_PARAMS_POSITION_MODE_ABSOLUTE);
				//motor_lower->set_position_to_drv(motor_priv->param.position,motor_priv->id,ELMO_PARAMS_POSITION_MODE_RELATIVE);
	  		  }else if(motor_priv->opmode == MOTOR_OPMODE_POSITION_RELATIVE){
	  			motor_lower->set_position_to_drv(motor_priv->param.position,motor_priv->id,ELMO_PARAMS_POSITION_MODE_RELATIVE);
	  		  }

	  	  }
	  		  break;

	  	  case PWRIOC_GET_DEVID:
	  	  {

			  if(motor_priv->id)
			  {
				  return motor_priv->id;
			  }else
			  {
				  return -ENXIO;//No such device or address;
			  }
	  	  }
	  		  break;


		  case PWRIOC_SEND_SYNC:
	  	  {
			  if(motor_priv->id)
			  {
				  motor_lower->send_sync_to_drv(motor_priv->param.position,motor_priv->id,(uint32_t *)arg);
			  }else
			  {
				  return -ENXIO;//No such device or address;
			  }
	  	  }
	  		  break;

	  	  default:
	  		  return -EINVAL;//Invalid argument;
	  		  break;
	  }

	  return OK;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct elmo_dops_s * elmodb4x_can_lower_init(void)
{
	return &g_elmo_dops;
}

/****************************************************************************
 * Name: elmodb4x_initialize
 *
 * Description:
 *   Initialize the selected elmodb4x Controller over canbus
 *
 * Input Parameters:
 *   config - The configuration structure passed by the board.
 *
 * Returned Value:
 *   Valid motor device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct motor_dev_s *elmodb4x_initialize(unsigned int motor_num ,struct can_dev_s *can_dev)
{
  FAR struct motor_dev_s *dev;

  /* Allocate a elmo4x Device structure */

  dev = (FAR struct motor_dev_s *)kmm_malloc(sizeof(struct motor_dev_s));
  if (dev == NULL)
    {
	  printf("ERROR: Failed to allocate instance of motor_dev_s!\n");
      return NULL;
    }

  dev->ops    = &g_motorops;
  dev->lower  = &g_elmo_dops;
  switch(motor_num){
  case 1:
	  dev->priv = &g_elmo_motor_1_priv;
	  dev->priv->priv = can_dev;
	  break;
  case 2:
	  dev->priv = &g_elmo_motor_2_priv;
	  break;
  case 3:
	  dev->priv = &g_elmo_motor_3_priv;
	  break;
  case 4:
	  dev->priv = &g_elmo_motor_4_priv;
	  break;
  case 5:
	  dev->priv = &g_elmo_motor_5_priv;
	  break;
  case 6:
	  dev->priv = &g_elmo_motor_6_priv;
	  break;
  default:
	  break;
  }
  return dev;
}

