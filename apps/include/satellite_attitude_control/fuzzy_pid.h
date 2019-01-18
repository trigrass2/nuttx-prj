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

#ifndef __APPS_SATELLITE_ATTITUDE_CONTROL_FUZZY_PID_H
#define __APPS_SATELLITE_ATTITUDE_CONTROL_FUZZY_PID_H

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
#define PB 100
#define PM 75
#define PS 55
#define ZO 30
#define NB 100
#define NM 75
#define NS 55


/****************************************************************************
 * Public Types
 ****************************************************************************/
struct fuzzy_pid_info_t
{
	//process data
	float P;								//比例输出
	float I;								//积分输出
	float D;								//微分输出
	float desired;							//PID输出
};

struct fuzzy_pid_params_t
{
	//PID parameters
	float kp;
	float ki;
	float kd;
	float kf;

	//filter parameters
	float fCut;								//一阶低通滤波器截止频率(Hz)

	//maximal integrator and output value
	float i_max;							//积分限制阀值
	float o_max;							//输出限制阀值

	//
	float i_threshold;						//积分分离阀值

};

struct fuzzy_pid_fuzzy_t
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

struct algorithm_fuzzy_pid_t
{
	//calculate data
	float integrator;						//积分值
	float last_error;						//上一次偏差值
	float last_derivative;					//上一次偏差微分值
	uint32_t last_t;						//上一次运行时间(系统绝对时间)

	//PID parameters
	struct fuzzy_pid_params_t params;		//需要保存到存储器的参数

	struct fuzzy_pid_fuzzy_t	tab;		//模糊表

	//process data
	struct fuzzy_pid_info_t info;			//过程观测量

	//control data
	bool enable;							//使能开关
};



/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Name: get_fuzzy_pid
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
float get_fuzzy_pid(struct algorithm_fuzzy_pid_t *pid, float error, float scaler);

#endif /* __APPS_SATELLITE_ATTITUDE_CONTROL_FUZZY_PID_H */
