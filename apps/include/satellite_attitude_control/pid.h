/****************************************************************************
 * app/include/satellite_attitude_control/pid.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_SATELLITE_ATTITUDE_CONTROL_PID_H
#define __APPS_SATELLITE_ATTITUDE_CONTROL_PID_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <math.h>
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
struct pid_info_t
{
	//process data
	float P;								//proportion result
	float I;								//integral result
	float D;								//derivative result
	float desired;							//PID result
};

struct pid_params_t
{
	//PID parameters
	float kp;
	float ki;
	float kd;
	float kf;

	//filter parameters
	float fCut;								//RC filter cut-off frequency

	//maximal integrator and output value
	float i_max;							//maximal integrator
	float o_max;							//maximal output value

};

struct pid_fuzzy_t
{
  int 	row;								//模糊PID,偏差行
  int 	col;								//模糊PID,微分列
  float ptab[10][10];						//模糊PID,比例权重控制表(行:偏差,列微分)
  float itab[10][10];						//模糊PID,积分权重控制表(行:偏差,列微分)
  float dtab[10][10];						//模糊PID,微分权重控制表(行:偏差,列微分)
  float Edot[10];							//模糊PID,偏差段
  float ECdot[10];							//模糊PID,微分段
  float kp;
  float ki;
  float kd;
};

struct algorithm_pid_t
{
	//calculate data
	float integrator;						//integrator
	float last_error;						//last error
	float last_derivative;					//last derivative error
	uint32_t axis_id;
	uint32_t last_t;						//last process absolute time.

	//adrc parameters
	struct adrc_data *adrc_params;		    //this data is for adrc control algorithm
	struct adrc_td *td_params;

	float * iir_filter_aFilt;			    //三阶IIR滤波器a系数
	float * iir_filter_bFilt;               //三阶IIR滤波器b系数

	//PID parameters
	struct pid_params_t params;				//this data will program into storages

	//process data
	struct pid_info_t info;					//all calculate data

	//control data
	bool enable;							//enable PID
};



/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: get_pid
 *
 * Description:
 *   Generic PID algorithm, with EEPROM-backed storage of constants.
 *
 * Input Parameters:
 *   pid    - pid-specific data
 *   error  - error of the target and feedback
 *   scaler - scale the P 、I and D components
 *
 * Returned Value:
 *   the result of PID algorithm.
 *
 ****************************************************************************/
float get_pid(struct algorithm_pid_t *pid, float error, float scaler);

float get_adrc_pid(struct algorithm_pid_t *pid, float in_need, float out_system, float scaler);

#endif /* __APPS_SATELLITE_ATTITUDE_CONTROL_PID_H */
