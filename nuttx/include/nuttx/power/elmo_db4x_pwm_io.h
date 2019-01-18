/****************************************************************************
 * include/nuttx/power/elmo_db4x_pwm_io.h
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

#ifndef __INCLUDE_NUTTX_DRIVERS_POWER_ELMO_DB4X_PWM_IO_H
#define __INCLUDE_NUTTX_DRIVERS_POWER_ELMO_DB4X_PWM_IO_H

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
#include <nuttx/drivers/pwm.h>

#ifdef CONFIG_MOTOR_ELMO_DB4X

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/


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

/*
 * elmo uper-half, platform-specific logic.
 */
struct motor_pwm_io_param_s
{
  bool      initialized;
  FAR char *devpath;
#ifdef CONFIG_PWM_MULTICHAN
  uint8_t   channels[CONFIG_PWM_NCHANNELS];
  int32_t   duties[CONFIG_PWM_NCHANNELS];
#else
  uint8_t   duty;
#endif
  uint32_t  frequency;
#ifdef CONFIG_PWM_PULSECOUNT
  uint32_t  count;
#endif
};

/*
 * elmo lower-half, platform-specific logic.
 */
struct motor_pwm_io_dev_s
{
	FAR struct pwm_info_s	*pwm_info;

    FAR struct pwm_lowerhalf_s *pwm_dev;

    CODE void (*gpiowrite)(uint32_t pinset, bool value);

    FAR uint32_t pinset[CONFIG_PWM_NCHANNELS];
};


/****************************************************************************
 * Name: elmodb4x_pwm_io_initialize
 ****************************************************************************/

FAR struct motor_dev_s *elmo_db4x_pwm_io_initialize(unsigned int motor_num);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MOTOR_ELMO_DB4X */
#endif /* __INCLUDE_NUTTX_DRIVERS_POWER_ELMO_DB4X_PWM_IO_H */
