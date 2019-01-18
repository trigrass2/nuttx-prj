/******************************************************************************
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Bob Feretich <bob.feretich@rafresearch.com>
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
 ******************************************************************************/

#ifndef __INCLUDE_SPSA_PLUS_ALGORITHM_H
#define __INCLUDE_SPSA_PLUS_ALGORITHM_H

#define SIGNAL_POWER_MIN	(-100)

//mode
enum spsa_step
{
	UPDATE_DISTRITUTION = 0,
	UPDATE_ESTIMATED_POSITION,
};

enum{
	SPSA_PLUS_AXIS_YAW = 0,
	SPSA_PLUS_AXIS_PITCH,
	SPSA_PLUS_AXIS_ALL ,
};

struct  spsa_params_t{
	/* 衰减参数 */
	float ak;								/* 估计步进系数 */
	float a;								/* 估计步进系数,衰减起点控制参数 */
	float big_a;							/* 估计步进系数,衰减起点控制参数 */
	float alpha;							/* 估计步进系数,衰减周期控制参数 */
	float ck;								/* 随机扰动步进系数 */
	float c;								/* 随机扰动步进系数 */
	float gamma;							/* 随机扰动步进系数,衰减周期控制参数 */
	float b;

	float p1;
	float p2;
	float p3;

	float k_az;
	float k_el;
	float S_max;
	float single_range;
	float single_range_max;
	float single_range_min;
};

/*
 * 扰动输出参数
 */
struct spsa_disturbance_t{
	float delta_k;							/* 随机扰动步进值 */
	float theta_k;							/* 估计步进值 */
	float out_put;							/* 输出值 */
	long  k;								/* 迭代记数 */
	float grade;
};

/*
 * 反馈(信号强度)数据
 */
struct  spsa_signal_power_t{
	float head;								/* 当前(t1时刻)信号强度 */
	float tile;								/* 上刻(t0时刻)信号强度 */
};

/*
 * 目标(位置)数据
 */
struct  spsa_target_position_t{
	float roll;
	float pitch;
	float yaw;
};

/*
 * SPSA算法数据结构
 */
struct  spsa_algorithm_t{

	int run_step;

	int axis;

	struct spsa_target_position_t target_position;

	/* 衰减参数 */
	struct spsa_params_t params;			/* 衰减参数  */

	/* 反馈(信号强度)参数 */
	struct spsa_signal_power_t signal_power;/* 反馈(信号强度)参数  */

	/* 扰动控制量 */
	struct spsa_disturbance_t disturbance;	/* 扰动控制量 */
};



/****************************************************************************
 * Name: spsa_plus_run
 *
 * Description:
 *   calculate the loss value.
 *
 * Input Parameters:
 *   pspsa_params - pointer of SPSA algorithm structure.
 *   pitch		  - antenna beam current pitch.
 *   yaw		  - antenna beam current yaw.
 *   power		  - antenna beacon power.
 *   stable       - antenna attitude control status.
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/
void spsa_plus_run(void *pspsa_params, float pitch, float yaw, float power, bool stable);

#endif /* __INCLUDE_SPSA_PLUS_ALGORITHM_H */

