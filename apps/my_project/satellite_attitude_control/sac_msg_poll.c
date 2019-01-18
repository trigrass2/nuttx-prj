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

#include <math.h>
#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <debug.h>
#include <errno.h>

#include <satellite_attitude_control/sac_beam_convert.h>
#include <satellite_attitude_control/message_handle.h>
#include <satellite_attitude_control/satellite_attitude_control.h>
#include <satellite_attitude_control/params_storage.h>
#include <satellite_attitude_control/pid.h>

/****************************************************************************
 * Name: do_orb_msg_poll
 *
 * Description:
 *   poll the uorb message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct
 *
 * Returned Value:
 *   poll result.
 *
 ****************************************************************************/

int do_orb_msg_poll(void * priv)
{
	struct sat_att_ctrl_t  *sac = priv;

	/*
	 * 设定poll读取目标
	 * 1.位置找零的时候目标为电机状态
	 * 2.姿态控制的时候目标为姿态状态
	 * 3.默认以姿态为控制周期
	 */
	if(MODE_CASE(sac->mode) == MODE_CASE(INIT)){
		sac->fds.fd 		= sac->sub_motors_fb->orb_fd;
		sac->fds.events 	= POLLIN;
	}else {
		sac->fds.fd 		= sac->sub_attitude->orb_fd;
		sac->fds.events 	= POLLIN;
	}


	/* poll all subscribed uorb message */
	int poll_ret = poll(&(sac->fds), 1, 500);

	/* 处理poll结果 */
	if (poll_ret < 0) {

		/* 错误异常,必须处理 */
		syslog(LOG_ERR,"ERROR:failed to poll attitude data: %d\n", poll_ret);
	}
	/* 超时异常 */
	else if(poll_ret == 0){

		syslog(LOG_ERR,"ERROR:timeout to poll attitude data: %d\n", poll_ret);
	}
	/* 检查需要更新的数据,执行对应逻辑 */
	else{

		bool update = false;

		/*
		 * 检查IMU信息
		 */
		orb_check(sac->sub_attitude->orb_fd,&update);
		if(update){

			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(imu_data_uorb),sac->sub_attitude->orb_fd, &(sac->sub_attitude->orb_data));
		}

		/*
		 * 检查姿态目标信息
		 */
		orb_check(sac->sub_attitude_sp->orb_fd,&update);
		if(update){

			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(attitude_set_point_uorb),sac->sub_attitude_sp->orb_fd, &(sac->sub_attitude_sp->orb_data));
			sac->cmd->ned_roll  = sac->sub_attitude_sp->orb_data.roll_sp;
			sac->cmd->ned_pitch = sac->sub_attitude_sp->orb_data.pitch_sp;
			sac->cmd->ned_yaw   = sac->sub_attitude_sp->orb_data.yaw_sp;
		}

		/*
		 * 检查PID参数信息
		 */
		orb_check(sac->sub_pid_params->orb_fd,&update);
		if(update){

			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(pc_PID_cmd_uorb),sac->sub_pid_params->orb_fd, &(sac->sub_pid_params->orb_data));

			/* 读取PID参数操作 */
			if(sac->sub_pid_params->orb_data.op_mode == 0x02){

				/* 信标环 */
				if(sac->sub_pid_params->orb_data.id == 0xA0){
				}
				/* 极化环 */
				else if(sac->sub_pid_params->orb_data.id == 0xA1){

				}
				/* 极化环2 */
				else if(sac->sub_pid_params->orb_data.id == 0xA2){

				}
				/* yaw */
				else if(sac->sub_pid_params->orb_data.id == 0xA3){
					sac->pub_pid_params_fb->msg.id 		= sac->sub_pid_params->orb_data.id;
					sac->pub_pid_params_fb->msg.enable 	= sac->rxatt_yaw->enable;
					sac->pub_pid_params_fb->msg.Kp 		= sac->rxatt_yaw->params.kp;
					sac->pub_pid_params_fb->msg.Ki 		= sac->rxatt_yaw->params.ki;
					sac->pub_pid_params_fb->msg.Kd 		= sac->rxatt_yaw->params.kd;
					sac->pub_pid_params_fb->msg.Kf 		= sac->rxatt_yaw->params.kf;
					sac->pub_pid_params_fb->msg.fCut  	= sac->rxatt_yaw->params.fCut;
					sac->pub_pid_params_fb->msg.iMax 	= sac->rxatt_yaw->params.i_max;
					sac->pub_pid_params_fb->msg.oMax 	= sac->rxatt_yaw->params.o_max;
					orb_publish(ORB_ID(pc_PID_params_uorb), sac->pub_pid_params_fb->pub, &(sac->pub_pid_params_fb->msg));
				}
				/* yaw2 */
				else if(sac->sub_pid_params->orb_data.id == 0xA4){
					sac->pub_pid_params_fb->msg.id 		= sac->sub_pid_params->orb_data.id;
					sac->pub_pid_params_fb->msg.enable 	= sac->rxatt_yaw2->enable;
					sac->pub_pid_params_fb->msg.Kp 		= sac->rxatt_yaw2->params.kp;
					sac->pub_pid_params_fb->msg.Ki 		= sac->rxatt_yaw2->params.ki;
					sac->pub_pid_params_fb->msg.Kd 		= sac->rxatt_yaw2->params.kd;
					sac->pub_pid_params_fb->msg.Kf 		= sac->rxatt_yaw2->params.kf;
					sac->pub_pid_params_fb->msg.fCut  	= sac->rxatt_yaw2->params.fCut;
					sac->pub_pid_params_fb->msg.iMax 	= sac->rxatt_yaw2->params.i_max;
					sac->pub_pid_params_fb->msg.oMax 	= sac->rxatt_yaw2->params.o_max;
					orb_publish(ORB_ID(pc_PID_params_uorb), sac->pub_pid_params_fb->pub, &(sac->pub_pid_params_fb->msg));
				}
				/* 俯仰环 */
				else if(sac->sub_pid_params->orb_data.id == 0xA5){
					sac->pub_pid_params_fb->msg.id 		= sac->sub_pid_params->orb_data.id;
					sac->pub_pid_params_fb->msg.enable 	= sac->rxatt_pitch->enable;
					sac->pub_pid_params_fb->msg.Kp 		= sac->rxatt_pitch->params.kp;
					sac->pub_pid_params_fb->msg.Ki 		= sac->rxatt_pitch->params.ki;
					sac->pub_pid_params_fb->msg.Kd 		= sac->rxatt_pitch->params.kd;
					sac->pub_pid_params_fb->msg.Kf 		= sac->rxatt_pitch->params.kf;
					sac->pub_pid_params_fb->msg.fCut  	= sac->rxatt_pitch->params.fCut;
					sac->pub_pid_params_fb->msg.iMax 	= sac->rxatt_pitch->params.i_max;
					sac->pub_pid_params_fb->msg.oMax 	= sac->rxatt_pitch->params.o_max;
					orb_publish(ORB_ID(pc_PID_params_uorb), sac->pub_pid_params_fb->pub, &(sac->pub_pid_params_fb->msg));
				}
			}
			/* 写入PID参数操作到RAM/FLASH */
			else if(sac->sub_pid_params->orb_data.op_mode < 0x02 ){
				/* 信标环 */
				if(sac->sub_pid_params->orb_data.id == 0xA0){
				}
				/* 极化环 */
				else if(sac->sub_pid_params->orb_data.id == 0xA1){

				}
				/* 极化环2 */
				else if(sac->sub_pid_params->orb_data.id == 0xA2){

				}
				/* yaw */
				else if(sac->sub_pid_params->orb_data.id == 0xA3){
					sac->cmd->ned_yaw	  			= sac->sub_pid_params->orb_data.target_value;
					sac->rxatt_yaw->enable	  		= sac->sub_pid_params->orb_data.enable;
					sac->rxatt_yaw->params.kp 		= sac->sub_pid_params->orb_data.Kp;
					sac->rxatt_yaw->params.ki 		= sac->sub_pid_params->orb_data.Ki;
					sac->rxatt_yaw->params.kd 		= sac->sub_pid_params->orb_data.Kd;
					sac->rxatt_yaw->params.kf 		= sac->sub_pid_params->orb_data.Kf;
					sac->rxatt_yaw->params.fCut  	= sac->sub_pid_params->orb_data.fCut;
					sac->rxatt_yaw->params.i_max 	= sac->sub_pid_params->orb_data.iMax;
					sac->rxatt_yaw->params.o_max 	= sac->sub_pid_params->orb_data.iMin;
				}
				/* yaw2 */
				else if(sac->sub_pid_params->orb_data.id == 0xA4){
					sac->cmd->ned_yaw	  			= sac->sub_pid_params->orb_data.target_value;
					sac->rxatt_yaw2->enable	   		= sac->sub_pid_params->orb_data.enable;
					sac->rxatt_yaw2->params.kp 		= sac->sub_pid_params->orb_data.Kp;
					sac->rxatt_yaw2->params.ki 		= sac->sub_pid_params->orb_data.Ki;
					sac->rxatt_yaw2->params.kd 		= sac->sub_pid_params->orb_data.Kd;
					sac->rxatt_yaw2->params.kf 		= sac->sub_pid_params->orb_data.Kf;
					sac->rxatt_yaw2->params.fCut  	= sac->sub_pid_params->orb_data.fCut;
					sac->rxatt_yaw2->params.i_max 	= sac->sub_pid_params->orb_data.iMax;
					sac->rxatt_yaw2->params.o_max 	= sac->sub_pid_params->orb_data.iMin;
				}
				/* 俯仰环 */
				else if(sac->sub_pid_params->orb_data.id == 0xA5){
					sac->cmd->ned_pitch	  			= sac->sub_pid_params->orb_data.target_value;
					sac->rxatt_pitch->enable	  	= sac->sub_pid_params->orb_data.enable;
					sac->rxatt_pitch->params.kp 	= sac->sub_pid_params->orb_data.Kp;
					sac->rxatt_pitch->params.ki 	= sac->sub_pid_params->orb_data.Ki;
					sac->rxatt_pitch->params.kd 	= sac->sub_pid_params->orb_data.Kd;
					sac->rxatt_pitch->params.kf 	= sac->sub_pid_params->orb_data.Kf;
					sac->rxatt_pitch->params.fCut  	= sac->sub_pid_params->orb_data.fCut;
					sac->rxatt_pitch->params.i_max 	= sac->sub_pid_params->orb_data.iMax;
					sac->rxatt_pitch->params.o_max 	= sac->sub_pid_params->orb_data.iMin;
				}

				if(sac->sub_pid_params->orb_data.op_mode == 0x00){
					/*
					 * save received antenna pitch direction parameters.
					 */
					int nbytes = params_save(&(sac->rxatt_pitch->params), ADDR_ATT_RX_CTRL_PITCH, sizeof(struct pid_params_t));
					if(nbytes != sizeof(struct pid_params_t)){
						printf("failed to save received antenna pitch direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
					}

					// save received antenna yaw direction parameters.
					nbytes = params_save(&(sac->rxatt_yaw->params), ADDR_ATT_RX_CTRL_YAW, sizeof(struct pid_params_t));
					if(nbytes != sizeof(struct pid_params_t)){
						printf("failed to save received antenna yaw direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
					}

					// save received antenna yaw2 direction parameters.
					nbytes = params_save(&(sac->rxatt_yaw2->params), ADDR_ATT_RX_CTRL_YAW2, sizeof(struct pid_params_t));
					if(nbytes != sizeof(struct pid_params_t)){
						printf("failed to save received antenna yaw2 direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
					}

					// save received antenna polar direction parameters.
					nbytes = params_save(&(sac->rxatt_polar->params), ADDR_ATT_RX_CTRL_POLAR, sizeof(struct pid_params_t));
					if(nbytes != sizeof(struct pid_params_t)){
						printf("failed to save received antenna polar direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
					}

					// save received antenna polar2 direction parameters.
					nbytes = params_save(&(sac->rxatt_polar2->params), ADDR_ATT_RX_CTRL_POLAR2, sizeof(struct pid_params_t));
					if(nbytes != sizeof(struct pid_params_t)){
						printf("failed to save received antenna polar2 direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
					}

					// save received antenna T parameters.
					nbytes = params_save(&(sac->rxatt_T->params), ADDR_ATT_RX_CTRL_T, sizeof(struct pid_params_t));
					if(nbytes != sizeof(struct pid_params_t)){
						printf("failed to save received antenna polar2 direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
					}
				}
			}
		}

		/*
		 * 检查电机状态信息
		 */
		orb_check(sac->sub_motors_fb->orb_fd,&update);
		if(update){

			/* 拷贝最新的数据 */

			orb_copy(ORB_ID(motor_state_uorb), sac->sub_motors_fb->orb_fd, &(sac->sub_motors_fb->orb_data));

			/* L1位置信息 */
			if(sac->sub_motors_fb->orb_data.cmd[0] == 101 ){
				/* 将电机坐标转换为[-180,180] */
				if(sac->sub_motors_fb->orb_data.params[0].fdata > 180.0){
					sac->sub_motors_fb->position[0] = 360.0 - sac->sub_motors_fb->orb_data.params[0].fdata;
				}else{
					sac->sub_motors_fb->position[0] = -sac->sub_motors_fb->orb_data.params[0].fdata;
				}

			}

			/* L2位置信息 */
			if(sac->sub_motors_fb->orb_data.cmd[1] == 101 ){
				/* 将电机坐标转换为[-180,180] */
				if(sac->sub_motors_fb->orb_data.params[1].fdata > 180.0){
					sac->sub_motors_fb->position[1] = 360.0 - sac->sub_motors_fb->orb_data.params[1].fdata;
				}else{
					sac->sub_motors_fb->position[1] = -sac->sub_motors_fb->orb_data.params[1].fdata;
				}

			}

			/* L3位置信息 */
			if(sac->sub_motors_fb->orb_data.cmd[2] == 101 ){
				/* 将电机坐标转换为[-180,180] */
				if(sac->sub_motors_fb->orb_data.params[2].fdata > 180.0){
					sac->sub_motors_fb->position[2] = 360.0 - sac->sub_motors_fb->orb_data.params[2].fdata;
				}else{
					sac->sub_motors_fb->position[2] = -sac->sub_motors_fb->orb_data.params[2].fdata;
				}
			}

			/* 数据有效性检查 */
			if(   fabsf(sac->sub_motors_fb->orb_data.params[0].fdata) > 360\
				||fabsf(sac->sub_motors_fb->orb_data.params[1].fdata) > 360\
				||fabsf(sac->sub_motors_fb->orb_data.params[2].fdata) > 360){

				syslog(LOG_ERR,"[SAC]ERROR:motors feedback data is illegal.\n");

				return ERROR;
			}
		}

		/*
		 * 检查卫星信息消息
		 */
		orb_check(sac->sub_sat_params->orb_fd,&update);
		if(update){

			/* 拷贝最新的数据 */

			orb_copy(ORB_ID(pc_satellite_cmd_uorb), sac->sub_sat_params->orb_fd, &(sac->sub_sat_params->orb_data));

			sac->beam->params.L = sac->sub_sat_params->orb_data.L;
			sac->beam->params.A1 = sac->sub_sat_params->orb_data.A1;
			sac->beam->params.A2 = sac->sub_sat_params->orb_data.A2;
			sac->beam->params.A3 = sac->sub_sat_params->orb_data.A3;
			sac->beam->params.A4 = sac->sub_sat_params->orb_data.A4;
			sac->beam->params.A5 = sac->sub_sat_params->orb_data.A5;
			sac->beam->params.A6 = sac->sub_sat_params->orb_data.A6;

			sac->beam->params.M = sac->sub_sat_params->orb_data.M;
			sac->beam->params.B1 = sac->sub_sat_params->orb_data.B1;
			sac->beam->params.B2 = sac->sub_sat_params->orb_data.B2;
			sac->beam->params.B3 = sac->sub_sat_params->orb_data.B3;
			sac->beam->params.B4 = sac->sub_sat_params->orb_data.B4;
			sac->beam->params.B5 = sac->sub_sat_params->orb_data.B5;
			sac->beam->params.B6 = sac->sub_sat_params->orb_data.B6;

			sac->beam->params.N = sac->sub_sat_params->orb_data.N;
			sac->beam->params.C1 = sac->sub_sat_params->orb_data.C1;
			sac->beam->params.C2 = sac->sub_sat_params->orb_data.C2;
			sac->beam->params.C3 = sac->sub_sat_params->orb_data.C3;
			sac->beam->params.C4 = sac->sub_sat_params->orb_data.C4;
			sac->beam->params.C5 = sac->sub_sat_params->orb_data.C5;
			sac->beam->params.C6 = sac->sub_sat_params->orb_data.C6;

		}

	}

	return poll_ret;
}
