/****************************************************************************
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <nuttx/config.h>
#include <nuttx/init.h>

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>

#include <debug.h>
#include <errno.h>

#include <satellite_attitude_control/pid.h>
#include <satellite_attitude_control/fuzzy_pid.h>
#include <satellite_attitude_control/params_storage.h>
#include <satellite_attitude_control/message_handle.h>
#include <satellite_attitude_control/sac_beam_convert.h>
#include <satellite_attitude_control/satellite_attitude_control.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: control_stabilize_linear_pid
 *
 * Description:
 *   satellite attitude control.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int control_stabilize_linear_pid(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;
	int ret = 0;
	float error = 0.0;

	/*
	 * PITCH轴偏差量
	 * 设定以L2为基准,L1顺时针转动为正(当俯仰角表示是射束与x-y平面夹角时,L1与L2重合,此时仰角靠近90度)
	 * 电机PWM控制:PWM值为负,逆时针转动 PWM值为正,顺时针转动
	 * 所以这里当error为正的时候,实际L1应该顺时针运动
	 */
	error = sac->cmd->ned_pitch - sac->beam->attitude.ned_pitch;

	/*  执行射束姿态闭环控制逻辑 <俯仰使能><航向使能> */
	float out_pitch = get_pid(sac->rxatt_pitch,\
							  -error,\
							  1.0);

	/*
	 * YAW/YAW2轴偏差量
	 * 航向控制L1/L2同时逆时针转动,对应航向正向增加[0, 180]
	 * 航向控制L1/L2同时逆时针转动,对应航向负向增加[0,-180]
	 * 那么当error为正的时候,实际L1/L2应该逆时针运动,为了匹配大地坐标系,已经在射束航向角度转换成了正逻辑
	 * 所以这里那么当error为正的时候,实际L1/L2应该顺时针运动!!!
	 */
	float tmp_yaw_error = sac->cmd->ned_yaw - sac->beam->attitude.ned_yaw;

	/* 当目标值在当前值的左边 */
	if(tmp_yaw_error < 0.0){

		/* 如果相差角度超过180度,顺时针运行 */
		if(fabsf(tmp_yaw_error) > 180.0){
			error = 360.0 + tmp_yaw_error;
		}
		/* 如果相差角度小于180度,逆时针运行 */
		else{
			error = tmp_yaw_error;
		}
	}
	/* 当目标值在当前值的右边 */
	else{
		/* 如果相差角度超过180度,逆时针运行 */
		if(fabsf(tmp_yaw_error) > 180.0){
			error = tmp_yaw_error - 360.0;
		}
		/* 如果相差角度小于180度,顺时针运行 */
		else{
			error = tmp_yaw_error;
		}
	}

	/*
	 * 当T角小于20度采用yaw2控制
	 * 当T角大于20度采用yaw 控制
	 */
	if(sac->beam->attitude.T < 20.0){
		/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
		float out_yaw2 = get_pid(sac->rxatt_yaw2,\
								error,\
								1.0);			/* 缩放比例,输出值/输出限幅度 */

		sac->pub_pid_turning->msg.D		   = sac->rxatt_yaw2->info.D;
	}else{
		/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
		float out_yaw = get_pid(sac->rxatt_yaw,\
								error,\
								1.0);			/* 缩放比例,输出值/输出限幅度 */
		sac->pub_pid_turning->msg.D		   = sac->rxatt_yaw->info.D;
	}


	/*
	 * POLAR轴偏差量
	 * 极化控制L3相对于L2顺时针转动,对应计划补偿角度[0, 180]
	 * L3的转动范围[-180,180],在保持L2转动的基础上在相对运动AP角度
	 * L3 = L2 + 45.0° - AP
	 */
	float tmp_polar_error = sac->cmd->ned_roll - sac->beam->attitude.ned_roll;

	/* 当目标值在当前值的左边 */
	if(tmp_polar_error < 0.0){

		/* 如果相差角度超过180度,顺时针运行 */
		if(fabsf(tmp_polar_error) > 180.0){
			error = 360.0 + tmp_polar_error;
		}
		/* 如果相差角度小于180度,逆时针运行 */
		else{
			error = tmp_polar_error;
		}
	}
	/* 当目标值在当前值的右边 */
	else{
		/* 如果相差角度超过180度,逆时针运行 */
		if(fabsf(tmp_polar_error) > 180.0){
			error = tmp_polar_error - 360.0;
		}
		/* 如果相差角度小于180度,顺时针运行 */
		else{
			error = tmp_polar_error;
		}
	}
	/*
	 * 当T角小于35度采用POLAR2控制
	 * 当T角大于35度采用POLAR 控制
	 */
	if(sac->beam->attitude.T < 35.0){
		/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
		float out_polar2 = get_pid(sac->rxatt_polar,\
								   error,\
								   1.0);		/* 缩放比例,输出值/输出限幅度 */

		sac->pub_pid_turning->msg.P		   = sac->rxatt_polar2->info.D;
	}else{
		/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
		float out_polar = get_pid(sac->rxatt_polar2,\
								  error,\
								  1.0);			/* 缩放比例,输出值/输出限幅度 */
		sac->pub_pid_turning->msg.P		   = sac->rxatt_polar->info.D;
	}

	/* 发送整定参数消息 */
	sac->pub_pid_turning->msg.axis = 1;
	sac->pub_pid_turning->msg.achieved = sac->rxatt_pitch->info.desired;
	sac->pub_pid_turning->msg.desired  = sac->cmd->ned_pitch;
	sac->pub_pid_turning->msg.I		   = sac->rxatt_pitch->last_error;
	sac->pub_pid_turning->msg.FF	   = out_pitch;

	orb_publish(ORB_ID(pid_turning_data_uorb), sac->pub_pid_turning->pub, &(sac->pub_pid_turning->msg));

	return OK;
}



/****************************************************************************
 * Name: control_stabilize_adrc
 *
 * Description:
 *   satellite attitude control.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int control_stabilize_adrc(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;
	int ret = 0;
	float error = 0.0;
	static int run_roll = 0;
	static int run_pitch = 0;
	static int run_yaw = 0;

#if 0
	run_yaw ++;
	if(run_yaw == 10)
	{
		run_yaw = 0;

		/*
		 * YAW/YAW2轴偏差量
		 * 航向控制L1/L2同时逆时针转动,对应航向正向增加[0, 180]
		 * 航向控制L1/L2同时逆时针转动,对应航向负向增加[0,-180]
		 * 那么当error为正的时候,实际L1/L2应该逆时针运动,为了匹配大地坐标系,已经在射束航向角度转换成了正逻辑
		 * 所以这里那么当error为正的时候,实际L1/L2应该顺时针运动!!!
		 */
		float tmp_yaw_error = sac->cmd->ned_yaw - sac->beam->attitude.ned_yaw;

		/* 当目标值在当前值的左边 */
		if(tmp_yaw_error < 0.0){
			/* 如果相差角度超过180度,顺时针运行 */
			if(fabsf(tmp_yaw_error) > 180.0){
				error = 360.0 + tmp_yaw_error;
			}
			/* 如果相差角度小于180度,逆时针运行 */
			else{
				error = tmp_yaw_error;
			}
		}
		/* 当目标值在当前值的右边 */
		else{
			/* 如果相差角度超过180度,逆时针运行 */
			if(fabsf(tmp_yaw_error) > 180.0){
				error = tmp_yaw_error - 360.0;
			}
			/* 如果相差角度小于180度,顺时针运行 */
			else{
				error = tmp_yaw_error;
			}
		}

		/*
		 * 当T角小于20度采用yaw2控制
		 * 当T角大于20度采用yaw 控制
		 */
		if(sac->beam->attitude.T < 20.0){
			/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
			float out_yaw2 = get_pid(sac->rxatt_yaw2,\
									error,\
									1.0);			/* 缩放比例,输出值/输出限幅度 */

			sac->pub_pid_turning->msg.D = out_yaw2;
		}else{
			/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
			float out_yaw = get_pid(sac->rxatt_yaw,\
									error,\
									1.0);			/* 缩放比例,输出值/输出限幅度 */
			sac->pub_pid_turning->msg.D = out_yaw;
		}


		/*
		 * POLAR轴偏差量
		 * 极化控制L3相对于L2顺时针转动,对应计划补偿角度[0, 180]
		 * L3的转动范围[-180,180],在保持L2转动的基础上在相对运动AP角度
		 * L3 = L2 + 45.0° - AP
		 */
		float tmp_polar_error = sac->cmd->ned_roll - sac->beam->attitude.ned_roll;

		/* 当目标值在当前值的左边 */
		if(tmp_polar_error < 0.0){

			/* 如果相差角度超过180度,顺时针运行 */
			if(fabsf(tmp_polar_error) > 180.0){
				error = 360.0 + tmp_polar_error;
			}
			/* 如果相差角度小于180度,逆时针运行 */
			else{
				error = tmp_polar_error;
			}
		}
		/* 当目标值在当前值的右边 */
		else{
			/* 如果相差角度超过180度,逆时针运行 */
			if(fabsf(tmp_polar_error) > 180.0){
				error = tmp_polar_error - 360.0;
			}
			/* 如果相差角度小于180度,顺时针运行 */
			else{
				error = tmp_polar_error;
			}
		}
		/*
		 * 当T角小于35度采用POLAR2控制
		 * 当T角大于35度采用POLAR 控制
		 */
		if(sac->beam->attitude.T < 35.0){
			/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
			float out_polar2 = get_pid(sac->rxatt_polar,\
									   error,\
									   1.0);		/* 缩放比例,输出值/输出限幅度 */

			sac->pub_pid_turning->msg.P	= out_polar2;
		}else{
			/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
			float out_polar = get_pid(sac->rxatt_polar2,\
									  error,\
									  1.0);			/* 缩放比例,输出值/输出限幅度 */
			sac->pub_pid_turning->msg.P	= out_polar;
		}

	}
#endif

	run_pitch ++;
	if(run_pitch == 5)
	{
		run_pitch = 0;
		/*
		* PITCH轴偏差量
		* 设定以L2为基准,L1顺时针转动为正(当俯仰角表示是射束与x-y平面夹角时,L1与L2重合,此时仰角靠近90度)
		* 电机PWM控制:PWM值为负,逆时针转动 PWM值为正,顺时针转动
		* 所以这里当error为正的时候,实际L1应该顺时针运动
		*/
		error = sac->cmd->ned_pitch - sac->beam->attitude.ned_pitch;

		// /*  执行射束姿态闭环控制逻辑 <俯仰使能><航向使能> */
		// float out_pitch = get_pid(sac->rxatt_pitch,\
		// 						  -error,\
		// 						  1.0);

		float out_pitch = get_adrc_pid(sac->rxatt_pitch,sac->cmd->ned_pitch,sac->beam->attitude.ned_pitch,1.0);
		sac->rxatt_pitch->info.desired = out_pitch;
	}

	run_roll++;
	if(run_roll == 1)
	{
		run_roll = 0;
		/*执行roll环控制*/

	}


	/* 发送整定参数消息 */
	sac->pub_pid_turning->msg.axis = 1;
	sac->pub_pid_turning->msg.achieved = sac->rxatt_pitch->info.desired;
	sac->pub_pid_turning->msg.desired  = sac->cmd->ned_pitch;
	sac->pub_pid_turning->msg.I		   = sac->rxatt_pitch->info.desired;
	sac->pub_pid_turning->msg.FF	   = 0;

	orb_publish(ORB_ID(pid_turning_data_uorb), sac->pub_pid_turning->pub, &(sac->pub_pid_turning->msg));

	return OK;
}

/****************************************************************************
 * Name: control_stabilize_fuzzy_pid
 *
 * Description:
 *   satellite attitude control with fuzzy PID.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int control_stabilize_fuzzy_pid(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;
	int ret = 0;
	float error = 0.0;
	static int run_roll = 0;
	static int run_pitch = 0;
	static int run_yaw = 0;

	run_yaw ++;
	if(run_yaw == 1)
	{
		run_yaw = 0;

		/*
		 * YAW轴偏差量
		 * 航向控制L1/L2同时顺时针转动,对应航向正向增加[0, 180]
		 * 航向控制L1/L2同时逆时针转动,对应航向负向增加[0,-180]
		 * 那么当error为正的时候,实际L1/L2应该顺时针运动
		 * 所以这里那么当error为正的时候,实际L1/L2应该顺时针运动!!!
		 */
		float tmp_yaw_error = sac->cmd->ned_yaw - sac->beam->attitude.ned_yaw;

		/* 当目标值在当前值的右边 */
		if(tmp_yaw_error < 0.0){
			/* 如果相差角度超过180度,顺时针运行 */
			if(fabsf(tmp_yaw_error) > 180.0){
				error = 360.0 + tmp_yaw_error;
			}
			/* 如果相差角度小于180度,逆时针运行 */
			else{
				error = tmp_yaw_error;
			}
		}
		/* 当目标值在当前值的左边 */
		else{
			/* 如果相差角度超过180度,逆时针运行 */
			if(fabsf(tmp_yaw_error) > 180.0){
				error = tmp_yaw_error - 360.0;
			}
			/* 如果相差角度小于180度,顺时针运行 */
			else{
				error = tmp_yaw_error;
			}
		}

		/*
		 * yaw 控制
		 */

		/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
		get_fuzzy_pid(sac->rx_yaw,\
					  -error,\
					  1.0);			/* 缩放比例,输出值/输出限幅度 */


		/*
		 * POLAR 控制
		 */
		/*执行roll环控制*/
		/*
		* POLAR轴偏差量
		* 设定以L2为基准,L3顺时针转动为正
		* 电机PWM控制:PWM值为负,逆时针转动 PWM值为正,顺时针转动
		* L2_L3范围是［-180,180],需要确保L3始终比L2超前(45.0 - sac->beam->attitude.t_roll)
		* 所以当error为正的时候表示L3超期了目标,所以电机需要反转
		*/
		error = sac->beam->attitude.L2_L3 - (45.0 - sac->beam->attitude.t_roll);

		 /*  执行射束姿态闭环控制逻辑 <俯仰使能><航向使能> */
		 get_fuzzy_pid(sac->rx_polar,\
				       -error,\
					   1.0);
	}

	run_pitch ++;
	if(run_pitch == 1)
	{
		run_pitch = 0;
		/*
		* PITCH轴偏差量
		* 设定以L2为基准,L1顺时针转动为正(当俯仰角表示是射束与x-y平面夹角时,L1与L2重合,此时仰角靠近90度)
		* 电机PWM控制:PWM值为负,逆时针转动 PWM值为正,顺时针转动
		*/
		error = sac->cmd->ned_pitch - sac->beam->attitude.ned_pitch;

		/*  执行射束姿态闭环控制逻辑 <俯仰使能><航向使能> */
		get_fuzzy_pid(sac->rx_pitch,\
				      -error,\
					  1.0);
	}


	/* 发送整定参数消息 */
	if(sac->pub_pid_turning->msg.axis == 1){
		sac->pub_pid_turning->msg.achieved = sac->rx_polar->last_error;
		sac->pub_pid_turning->msg.desired  = sac->rx_polar->info.desired;
		sac->pub_pid_turning->msg.P		   = sac->rx_polar->tab.kp;
		sac->pub_pid_turning->msg.I		   = sac->rx_polar->tab.ki;
		sac->pub_pid_turning->msg.D 	   = sac->rx_polar->tab.kd;
		sac->pub_pid_turning->msg.FF	   = sac->rx_polar->last_derivative;
	}else if(sac->pub_pid_turning->msg.axis == 2){
		sac->pub_pid_turning->msg.achieved = sac->rx_pitch->last_error;
		sac->pub_pid_turning->msg.desired  = sac->rx_pitch->info.desired;
		sac->pub_pid_turning->msg.P		   = sac->rx_pitch->tab.kp;
		sac->pub_pid_turning->msg.I		   = sac->rx_pitch->tab.ki;
		sac->pub_pid_turning->msg.D 	   = sac->rx_pitch->tab.kd;
		sac->pub_pid_turning->msg.FF	   = sac->rx_pitch->last_derivative;
	}else if(sac->pub_pid_turning->msg.axis == 3){
		sac->pub_pid_turning->msg.achieved = sac->rx_yaw->last_error;
		sac->pub_pid_turning->msg.desired  = sac->rx_yaw->info.desired;
		sac->pub_pid_turning->msg.P		   = sac->rx_yaw->tab.kp;
		sac->pub_pid_turning->msg.I		   = sac->rx_yaw->tab.ki;
		sac->pub_pid_turning->msg.D 	   = sac->rx_yaw->tab.kd;
		sac->pub_pid_turning->msg.FF	   = sac->rx_yaw->last_derivative;
	}else{
		sac->pub_pid_turning->msg.achieved = sac->rx_pitch->last_error;
		sac->pub_pid_turning->msg.desired  = sac->rx_pitch->info.desired;
		sac->pub_pid_turning->msg.P		   = sac->rx_pitch->tab.kp;
		sac->pub_pid_turning->msg.I		   = sac->rx_pitch->tab.ki;
		sac->pub_pid_turning->msg.D 	   = sac->rx_pitch->tab.kd;
		sac->pub_pid_turning->msg.FF	   = sac->rx_pitch->last_derivative;
	}


	orb_publish(ORB_ID(pid_turning_data_uorb), sac->pub_pid_turning->pub, &(sac->pub_pid_turning->msg));

	return OK;
}


/****************************************************************************
 * Name: control_t_stabilize
 *
 * Description:
 *   satellite attitude control.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int control_t_stabilize(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;
	int ret = 0;
	float error = 0.0;
	static int run_roll = 0;
	static int run_pitch = 0;
	float temp_l1_position = 0.0;
	float temp_l2_position = 0.0;
	float temp_angle_T	  = 0.0;

	/*
	 * 设置T角的值为L1相对L2的夹角
	 * 因为T不能为负,所以在正常控制中L1的角度始终大于L2的角度!!!
	 * 只有在初始化找零的时候可能出现L2大于L1(应为L2可能出现在180.0附近)
	 */
	temp_l1_position = sac->sub_motors_fb->position[0];//-180 ~ +180
	temp_l2_position = sac->sub_motors_fb->position[1];//-180 ~ +180

	/*
	 * 区分同号和异号
	 */

	/*
	 * 如果是异号
	 * 出现L1与L2差值符号反向,差值突变两种问题
	 */
	if((temp_l1_position * temp_l2_position) < 0){

		/* 差值的绝对值大于180.0度,出现差值突变现象 */
		if((temp_l1_position - temp_l2_position) >= 180.0){

			temp_angle_T =  (temp_l1_position - temp_l2_position) - 360.0;

		}
		/* 差值的绝对值小于-180.0度,出现差值突变并且符号反向现象 */
		else if((temp_l1_position - temp_l2_position) <= -180.0){

			temp_angle_T = 360.0 + (temp_l1_position - temp_l2_position);

		}
		else{

		/* 差值的绝对值小于180.0度 */
			temp_angle_T = temp_l1_position - temp_l2_position;
		}

	}
	/* 如果是同号 */
	else{

		/* 差值的绝对值一定小于180.0度 */
		temp_angle_T = temp_l1_position - temp_l2_position;

	}

//	/* 闭环T角 */
//	error = sac->cmd->T - temp_angle_T;
//
//	/*  执行T角闭环控制逻辑 <俯仰使能><航向使能> */
//	float out_t = get_pid(sac->rxatt_T,\
//							  error,\
//							  1.0);


	return OK;
}
