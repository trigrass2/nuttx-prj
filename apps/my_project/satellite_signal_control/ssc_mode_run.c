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
#include <math.h>

#include <debug.h>
#include <errno.h>

#include <satellite_signal_control/ssc_msg_handle.h>
#include <satellite_signal_control/spsa_plus_algorithm.h>
#include <satellite_signal_control/satellite_signal_control.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#include <satellite_signal_control/normally_distributed_table.h>

static uint32_t iter_k;
static uint32_t pk = 0 ;
static uint32_t yk = 0 ;

static float params_c = 1.8;
static float params_a = 1.6;
static float params_A = 8.6;

static float params_alpha = 0.602;
static float params_gamma = 0.101;


static float delat_kp1 = 0;
static float delat_kp2 = 0;

static float delat_ky1 = 0;
static float delat_ky2 = 0;

static float power_p1 = 0;
static float power_p2 = 0;

static float power_y1 = 0;
static float power_y2 = 0;

static float grade_p = 0;
static float grade_y = 0;

#define PITCH_AXIS 0
#define YAW_AXIS 1
#define RAND_TABLE_SIZE (1000)

#define RUN_SIGNAL_SPSA_TEST 19

static inline float get_random(void)
{
	float tmp = normally_distributed_table[abs(rand())%RAND_TABLE_SIZE];

	if(tmp >= 0) 
		tmp = 1;
	else 
		tmp = -1;
	return tmp;
}

float get_power(void *psat_signal_ctrl)
{
	struct sat_signal_ctrl_t *ssc = psat_signal_ctrl;	
	float power;
	power = ssc->sub_beacon_info->orb_data.beacon_signal.power;
	return power;
}
void set_beam_attitude(void *psat_signal_ctrl,float angele,int axis)
{
	struct sat_signal_ctrl_t *ssc = psat_signal_ctrl;	

	if(axis == PITCH_AXIS)
	{
		ssc->target_satellite->postion.pitch = angele;
	}
	else if(axis == YAW_AXIS)
	{
		ssc->target_satellite->postion.yaw	= angele;
	}

	ssc->pub_attitude_sp->msg.roll_sp  		= 0.0;
	ssc->pub_attitude_sp->msg.pitch_sp 		= ssc->target_satellite->postion.pitch;
	ssc->pub_attitude_sp->msg.yaw_sp   		= ssc->target_satellite->postion.yaw;
	ssc->pub_attitude_sp->msg.beacon_power 	= ssc->sub_beacon_info->orb_data.beacon_signal.power;
	orb_publish(ORB_ID(attitude_set_point_uorb), ssc->pub_attitude_sp->pub, &(ssc->pub_attitude_sp->msg));
}

static 	float p_ck,p_ak;
static 	float y_ck,y_ak;

static 	float estimate_pitch;
static 	float estimate_yaw;

static 	float last_pitch;
static 	float last_yaw;

#define DELAY_TIM 6

float spsa_update_delta(void *psat_signal_ctrl,float curr_pitch,float curr_yaw)
{
	iter_k++;

	if(iter_k == 1)
	{
		last_pitch = curr_pitch;
		last_yaw = curr_yaw;

		p_ck = 	params_c/powf((pk+1),params_gamma);
		p_ak = 	params_a/powf((pk+params_A+1),params_alpha);

		y_ck = 	params_c/powf((yk+1),params_gamma);
		y_ak = 	params_a/powf((yk+params_A+1),params_alpha);

		//驱动两次波速变相
		delat_kp1 = get_random();
		delat_kp2 = get_random();

		delat_ky1 = get_random();
		delat_ky2 = get_random();

		//采集两次波速变相功率值
		set_beam_attitude(psat_signal_ctrl,last_pitch + p_ck * delat_kp1,PITCH_AXIS);

		printf("iter_k = %d \n",iter_k);
		return;
	}

	if(iter_k == DELAY_TIM)
	{
		power_p1 = get_power(psat_signal_ctrl);
		set_beam_attitude(psat_signal_ctrl,last_pitch - p_ck * delat_kp1,PITCH_AXIS);
		printf("iter_k = %d \n",iter_k);
		return;
	}
		
	if(iter_k == DELAY_TIM*2)
	{
		power_p2 = get_power(psat_signal_ctrl);
		set_beam_attitude(psat_signal_ctrl,last_yaw + y_ck * delat_ky1,YAW_AXIS);
		printf("iter_k = %d \n",iter_k);
		return;
	}

	if(iter_k == DELAY_TIM*3)
	{
		power_y1 =  get_power(psat_signal_ctrl);
		set_beam_attitude(psat_signal_ctrl,last_yaw + y_ck * delat_ky1,YAW_AXIS);
		printf("iter_k = %d \n",iter_k);
		return;
	}

	if(iter_k == DELAY_TIM*4)
	{
		power_y2 = get_power(psat_signal_ctrl);

		//计算梯度
		grade_p = (power_p1 - power_p2)/(p_ck*2*delat_kp1);
		grade_y = (power_y1 - power_y2)/(y_ck*2*delat_ky1);

		//更新下一次的估计值
		estimate_pitch = last_pitch + grade_p*p_ak;
		estimate_yaw = last_yaw + grade_y*y_ak;		
		//判断是否需要从新迭代
		set_beam_attitude(psat_signal_ctrl,estimate_pitch,PITCH_AXIS);
		printf("iter_k = %d \n",iter_k);
		return;
	}

	if(iter_k == DELAY_TIM*5)
	{
		if(get_power(psat_signal_ctrl) < -95)
			pk = 0;
		else
			pk++;

		set_beam_attitude(psat_signal_ctrl,estimate_yaw,YAW_AXIS);
		printf("iter_k = %d \n",iter_k);
		return;
	}

	if(iter_k == DELAY_TIM*6)
	{
		if(get_power(psat_signal_ctrl) < -95)
			yk = 0;
		else
			yk++;
		printf("iter_k = %d \n",iter_k);

		iter_k = 0;
		return;		
	}
	return;
}
/****************************************************************************
 * Name: mode_control_signal_run
 *
 * Description:
 *   satellite signale control initialize.
 *
 * Input Parameters:
 *   psat_signal_ctrl  - pointer of struct sat_signal_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int mode_control_signal_run(void *psat_signal_ctrl)
{
	struct sat_signal_ctrl_t *ssc = psat_signal_ctrl;
	int ret = 0;
	float tmp_error_pitch = 0.0,tmp_error_yaw = 0.0;

	/* 判断是否更新目标卫星位置 */
	if((ssc->target_satellite->postion.pitch != ssc->target_satellite->postion.last_pitch) ||\
	   (ssc->target_satellite->postion.yaw != ssc->target_satellite->postion.last_yaw)){

		/* 重新搜星 */
		ssc->mode = RUN_POSITION_SEARCH;

		ssc->spsa_pitch->run_step = UPDATE_DISTRITUTION;
		ssc->spsa_pitch->disturbance.k = 0;

		ssc->spsa_yaw->run_step = UPDATE_DISTRITUTION;
		ssc->spsa_yaw->disturbance.k = 0;

	}

	/* 更新上一次的目标卫星位置 */
	ssc->target_satellite->postion.last_pitch = ssc->target_satellite->postion.pitch;
	ssc->target_satellite->postion.last_yaw = ssc->target_satellite->postion.yaw;

	switch(ssc->mode){

		/*
		 * 搜寻目标卫星大致位置
		 */
		case RUN_POSITION_SEARCH:
		{
			/*
			 * 等待姿态进入目标位置
			 * 1.如果姿态稳定在1度以内,粗寻位完成
			 * 2.进入随机扰动搜索
			 */
//			tmp_error_pitch = ssc->target_satellite->postion.pitch - ssc->sub_attitude_control_status->orb_data.curr_pitch;
//			tmp_error_yaw = ssc->target_satellite->postion.yaw - ssc->sub_attitude_control_status->orb_data.curr_yaw;
//
//			/* 寻位判断 */
//			if((fabsf(tmp_error_pitch) < 0.1) && (fabsf(tmp_error_yaw) < 0.1)){
//
//				/* 寻位完成 */
//				syslog(LOG_INFO,"[SSC]search target satellite position completed.\n");
//
//				/* 开始随机扰动搜索 */
//				syslog(LOG_INFO,"[SSC]start antenna random disturbing...\n");
//
//				/* change state machine to RUN_DISTURBANCE_SEARCH */
//				// ssc->mode = RUN_DISTURBANCE_SEARCH_YAW;
//				ssc->mode = RUN_SIGNAL_SPSA_TEST;
//			}

			/*
			 * 发布控制量到orb数据总线
			 */
			ssc->pub_attitude_sp->msg.roll_sp  		= 0.0;
			ssc->pub_attitude_sp->msg.pitch_sp 		= ssc->target_satellite->postion.pitch;
			ssc->pub_attitude_sp->msg.yaw_sp   		= ssc->target_satellite->postion.yaw;
			ssc->pub_attitude_sp->msg.beacon_power 	= ssc->sub_beacon_info->orb_data.beacon_signal.power;

			orb_publish(ORB_ID(attitude_set_point_uorb), ssc->pub_attitude_sp->pub, &(ssc->pub_attitude_sp->msg));

		}break;
		
		case RUN_DISTURBANCE_SEARCH_YAW:
		{
			/*
			 * 执行搜星控制逻辑[YAW]
			 * 1.调用搜星逻辑,输出yaw控制量
			 */
			ssc->spsa_yaw->target_position.yaw   = ssc->target_satellite->postion.yaw;
			ssc->spsa_yaw->target_position.pitch = ssc->target_satellite->postion.pitch;

			spsa_plus_run(ssc->spsa_yaw,\
						ssc->sub_attitude_control_status->orb_data.curr_pitch,\
						ssc->sub_attitude_control_status->orb_data.curr_yaw,\
						ssc->sub_beacon_info->orb_data.beacon_signal.power,\
						ssc->sub_attitude_control_status->orb_data.stable);
			/*
			 * 发布控制量到orb数据总线
			 * 1.发布扰动量
			 * 2.发布估计位置
			 * 3.如果信标强度低于阀值,采用在目标卫星位置中心附近扰动
			 * 4.如果信标强度达到阀值,采用信标强度收敛控制
			 */
			if(ssc->sub_beacon_info->orb_data.beacon_signal.power < SIGNAL_POWER_MIN){
				ssc->spsa_yaw->disturbance.k 			= 0;
				ssc->pub_attitude_sp->msg.roll_sp  		= 0.0;
				ssc->pub_attitude_sp->msg.pitch_sp 		= ssc->spsa_pitch->target_position.pitch;
				ssc->pub_attitude_sp->msg.yaw_sp   		= ssc->spsa_pitch->target_position.yaw + ssc->spsa_yaw->disturbance.out_put;
				ssc->pub_attitude_sp->msg.beacon_power 	= ssc->sub_beacon_info->orb_data.beacon_signal.power;
			}else{
				ssc->pub_attitude_sp->msg.roll_sp  		= 0.0;
				ssc->pub_attitude_sp->msg.pitch_sp 		= ssc->sub_attitude_control_status->orb_data.curr_pitch;
				ssc->pub_attitude_sp->msg.yaw_sp   		= ssc->sub_attitude_control_status->orb_data.curr_yaw + ssc->spsa_yaw->disturbance.out_put;
				ssc->pub_attitude_sp->msg.beacon_power 	= ssc->sub_beacon_info->orb_data.beacon_signal.power;
			}

			orb_publish(ORB_ID(attitude_set_point_uorb), ssc->pub_attitude_sp->pub, &(ssc->pub_attitude_sp->msg));


			/*
			 * 当SPSA执行完UPDATE_NEXT_POSITION后,run_step就变为UPDATE_DISTRITUTION
			 * 这就意味着一个轴的扫描周期完成
			 * 所以这个时候需要切换轴扫描
			 */
			if (ssc->spsa_yaw->run_step == UPDATE_DISTRITUTION){
				ssc->mode = RUN_DISTURBANCE_SEARCH_PITCH;
			}

		}break;

		case RUN_DISTURBANCE_SEARCH_PITCH:
		{
			/*
			 * 执行搜星控制逻辑[PITCH]
			 * 1.调用搜星逻辑,输出pitch控制量
			 */
			ssc->spsa_pitch->target_position.yaw   = ssc->target_satellite->postion.yaw;
			ssc->spsa_pitch->target_position.pitch = ssc->target_satellite->postion.pitch;

			spsa_plus_run(ssc->spsa_pitch,\
						ssc->sub_attitude_control_status->orb_data.curr_pitch,\
						ssc->sub_attitude_control_status->orb_data.curr_yaw,\
						ssc->sub_beacon_info->orb_data.beacon_signal.power,\
						ssc->sub_attitude_control_status->orb_data.stable);
			/*
			 * 发布控制量到orb数据总线
			 * 1.发布扰动量
			 * 2.发布估计位置
			 * 3.如果信标强度低于阀值,采用在目标卫星位置中心附近扰动
			 * 4.如果信标强度达到阀值,采用信标强度收敛控制
			 */
			if(ssc->sub_beacon_info->orb_data.beacon_signal.power > SIGNAL_POWER_MIN){
				ssc->spsa_pitch->disturbance.k 			= 0;
				ssc->pub_attitude_sp->msg.roll_sp  		= 0.0;
				ssc->pub_attitude_sp->msg.pitch_sp 		= ssc->sub_attitude_control_status->orb_data.curr_pitch + ssc->spsa_pitch->disturbance.out_put;
				ssc->pub_attitude_sp->msg.yaw_sp   		= ssc->sub_attitude_control_status->orb_data.curr_yaw;
				ssc->pub_attitude_sp->msg.beacon_power 	= ssc->sub_beacon_info->orb_data.beacon_signal.power;
			}else{
				ssc->pub_attitude_sp->msg.roll_sp  		= 0.0;
				ssc->pub_attitude_sp->msg.pitch_sp 		= ssc->spsa_pitch->target_position.pitch + ssc->spsa_pitch->disturbance.out_put;
				ssc->pub_attitude_sp->msg.yaw_sp   		= ssc->spsa_pitch->target_position.yaw;
				ssc->pub_attitude_sp->msg.beacon_power 	= ssc->sub_beacon_info->orb_data.beacon_signal.power;
			}

			orb_publish(ORB_ID(attitude_set_point_uorb), ssc->pub_attitude_sp->pub, &(ssc->pub_attitude_sp->msg));

			/*
			 * 当SPSA执行完UPDATE_NEXT_POSITION后,run_step就变为UPDATE_DISTRITUTION
			 * 这就意味着一个轴的扫描周期完成
			 * 所以这个时候需要切换轴扫描
			 */
			if (ssc->spsa_pitch->run_step == UPDATE_DISTRITUTION){
				ssc->mode = RUN_DISTURBANCE_SEARCH_YAW;
			}
		}break;

		case RUN_SIGNAL_SPSA_TEST:
		{
			spsa_update_delta(ssc,\
							  ssc->sub_attitude_control_status->orb_data.curr_pitch,\
							  ssc->sub_attitude_control_status->orb_data.curr_yaw);
		}break;
		case RUN_SIGNAL_SEARCH:
		{
			/* 所有状态正常，进入信号跟踪控制模式 */
			syslog(LOG_INFO,"[SSC]enter mode RUN_SIGNAL_SEARCH.\n");
		}break;

		default:
		{
			;
		}break;
	}

	return OK;

}
//int mode_control_signal_run(void *psat_signal_ctrl)
//{
//	struct sat_signal_ctrl_t *ssc = psat_signal_ctrl;
//	int ret = 0;
//	float tmp_error_pitch = 0.0,tmp_error_yaw = 0.0;
//
//	/* 判断是否更新目标卫星位置 */
//	if((ssc->target_satellite->postion.pitch != ssc->target_satellite->postion.last_pitch) ||\
//	   (ssc->target_satellite->postion.yaw != ssc->target_satellite->postion.last_yaw)){
//
//		/* 重新搜星 */
//		ssc->mode 		= RUN_POSITION_SEARCH;
//		ssc->spsa->mode = PUBLISH_DISTRITUTION;
//		ssc->spsa->k	= 0;
//
//	}
//
//	/* 更新上一次的目标卫星位置 */
//	ssc->target_satellite->postion.last_pitch = ssc->target_satellite->postion.pitch;
//	ssc->target_satellite->postion.last_yaw = ssc->target_satellite->postion.yaw;
//
//	switch(ssc->mode){
//
//		/*
//		 * 搜寻目标卫星大致位置
//		 */
//		case RUN_POSITION_SEARCH:
//		{
//			/*
//			 * 等待姿态进入目标位置
//			 * 1.如果姿态稳定在1度以内,粗寻位完成
//			 * 2.进入随机扰动搜索
//			 */
//			tmp_error_pitch = ssc->target_satellite->postion.pitch - ssc->sub_attitude_control_status->orb_data.curr_pitch;
//			tmp_error_yaw = ssc->target_satellite->postion.yaw - ssc->sub_attitude_control_status->orb_data.curr_yaw;
//
//			/* 寻位判断 */
//			if((fabsf(tmp_error_pitch) < 0.2) && (fabsf(tmp_error_yaw) < 0.2)){
//
//				/* 寻位完成 */
//				syslog(LOG_INFO,"[SSC]search target satellite position completed.\n");
//
//				/* 开始随机扰动搜索 */
//				syslog(LOG_INFO,"[SSC]start antenna random disturbing...\n");
//
//				/* change state machine to RUN_DISTURBANCE_SEARCH */
//				ssc->mode = RUN_DISTURBANCE_SEARCH;
//			}
//
//			/*
//			 * 发布控制量到orb数据总线
//			 */
//			ssc->pub_attitude_sp->msg.roll_sp  		= 0.0;
//			ssc->pub_attitude_sp->msg.pitch_sp 		= ssc->target_satellite->postion.pitch;
//			ssc->pub_attitude_sp->msg.yaw_sp   		= ssc->target_satellite->postion.yaw;
//			ssc->pub_attitude_sp->msg.beacon_power 	= ssc->sub_beacon_info->orb_data.beacon_signal.power;
//
//			orb_publish(ORB_ID(attitude_set_point_uorb), ssc->pub_attitude_sp->pub, &(ssc->pub_attitude_sp->msg));
//
//		}break;
//
//		case RUN_DISTURBANCE_SEARCH:
//		{
//			/*
//			 * 执行搜星控制逻辑[YAW]
//			 * 1.调用搜星逻辑,输出yaw控制量
//			 */
//			ssc->spsa->az = ssc->target_satellite->postion.yaw;
//			ssc->spsa->el = ssc->target_satellite->postion.pitch;
//
//			spsa_run(ssc->spsa,\
//					ssc->sub_attitude_control_status->orb_data.curr_pitch,\
//					ssc->sub_attitude_control_status->orb_data.curr_yaw,\
//					ssc->sub_beacon_info->orb_data.beacon_signal.power,\
//					ssc->sub_attitude_control_status->orb_data.stable);
//			/*
//			 * 发布控制量到orb数据总线
//			 */
//			if(ssc->spsa->run_step = UPDATE_DISTRITUTION){
//				ssc->pub_attitude_sp->msg.roll_sp  		= 0.0;
//				ssc->pub_attitude_sp->msg.pitch_sp 		= ssc->spsa->theta_plus[AXIS_PITCH];
//				ssc->pub_attitude_sp->msg.yaw_sp   		= ssc->spsa->theta_plus[AXIS_YAW];
//				ssc->pub_attitude_sp->msg.beacon_power 	= ssc->sub_beacon_info->orb_data.beacon_signal.power;
//				orb_publish(ORB_ID(attitude_set_point_uorb), ssc->pub_attitude_sp->pub, &(ssc->pub_attitude_sp->msg));
//			}
//
//		}break;
//
//		case RUN_SIGNAL_SEARCH:
//		{
//			/* 所有状态正常，进入姿态控制模式 */
//			syslog(LOG_INFO,"[SSC]enter mode RUN_SIGNAL_CONTROL.\n");
//		}break;
//
//		default:
//		{
//			;
//		}break;
//	}
//
//	return OK;
//
//}


