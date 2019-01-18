/****************************************************************************
 * include/nuttx/power/elmo_db4x.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_ELMO_DB4X_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_ELMO_DB4X_H

/*
 * The elmo_db4x driver is split into two parts:
 *
 * 1) An "upper half", generic driver that provides the common elmo_db4x interface
 *    to application level code, and
 * 2) A "lower half", platform-specific driver that implements the low-level
 *    functionality eg.:
 *      - timer controls to implement the PWM signals,
 *      - analog peripherals configuration such as ADC, DAC and comparators,
 *      - control algorithm for elmo_db4x driver (eg. FOC control for BLDC)
 *
 * This 'upper-half' driver has been designed with flexibility in mind
 * to support all kinds of electric elmo_db4xs and their applications.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <nuttx/power/motor.h>
#include <nuttx/power/power_ioctl.h>
#include <nuttx/can/can.h>

#ifdef CONFIG_MOTOR_ELMO_DB4X

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
enum{
	VELOCITY = 1,
	ACCELERATE,
	DECELERATE,
	MODE_TYPE
};

/****************************************************************************
 * Public Data
 ****************************************************************************/
#define ELMO_PARAMS_POSITION_MODE_ABSOLUTE 		0
#define ELMO_PARAMS_POSITION_MODE_RELATIVE		1
#define ELMO_PARAMS_ACCELERATE_ANGLE_MAX		1800
#define ELMO_PARAMS_VELOCITY_ANGLE_MAX			720

#define ELMO_LIMITS_POSITION_ANGLE_MAX			1080
#define ELMO_LIMITS_POSITION_LINE_MAX			1000000000



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


/*
 * elmo lower-half, platform-specific logic.
 */

struct elmo_dops_s
{
  /* Configure elmo driver */

  CODE int (*set_params_to_drv)(FAR unsigned char ,FAR int, FAR void *);

  /* Initialize elmo driver */

  CODE void (*init_drv)(FAR unsigned char,struct can_dev_s *);

  /* Set elmo driver position */

  CODE int (*set_position_to_drv)(FAR float , FAR unsigned char, FAR int);

  /* Send elmo driver sync */

  CODE int (*send_sync_to_drv)(FAR float , FAR unsigned char, FAR void *);
};

/* source from elmo.c */
extern int Motor_ELMO_Parameter_Set(unsigned char module_id,int type,void *para);
extern void Elmo_Init(unsigned char module_id,struct can_dev_s *dev);
extern int Motor_Position_Move(float position,  unsigned char module_id, int position_mode);

extern int Motor_ELMO_State_Get(float position,  unsigned char module_id, uint32_t *para);

/****************************************************************************
 * Name: elmodb4x_initialize
 ****************************************************************************/

FAR struct motor_dev_s *elmodb4x_initialize(unsigned int motor_num ,struct can_dev_s *can_dev);
FAR struct elmo_dops_s *elmodb4x_can_lower_init(void);
#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DRIVERS_ELMO_DB4X */
#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_ELMO_DB4X_H */
