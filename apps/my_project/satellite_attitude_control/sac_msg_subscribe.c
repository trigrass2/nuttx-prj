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

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <debug.h>
#include <errno.h>

#include <satellite_attitude_control/message_handle.h>
#include <satellite_attitude_control/satellite_attitude_control.h>

/****************************************************************************
 * Name: do_orb_msg_subscribe
 *
 * Description:
 *   subscribe orb message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
int do_orb_msg_subscribe(void *priv)
{
	struct sat_att_ctrl_t  *sac = priv;
	memset(&(sac->sub_attitude->orb_data), 0, sizeof(sac->sub_attitude->orb_data));

	/*
	 * 订阅IMU的uORB消息,更新频率须大于100Hz
	 * 清除变量内存
	 * 订阅消息失败,syslog报错记录,然后退出线程
	 * 订阅消息成功,保存订阅fd
	 */

	sac->sub_attitude->orb_fd = orb_subscribe(ORB_ID(imu_data_uorb));

	if(sac->sub_attitude->orb_fd < 0){
		syslog(LOG_ERR,"[uORB] Failed to subscribe imu data.\n");
		return ERROR;
	}

	/*
	 * 订阅电机位置的uORB消息,更新频率须大于100Hz
	 * 清除变量内存
	 * 订阅消息失败,syslog报错记录,然后退出线程
	 * 订阅消息成功,保存订阅fd
	 */

	memset(&(sac->sub_motors_fb->orb_data), 0, sizeof(sac->sub_motors_fb->orb_data));

	sac->sub_motors_fb->orb_fd = orb_subscribe(ORB_ID(motor_state_uorb));

	if(sac->sub_motors_fb->orb_fd < 0){
		syslog(LOG_ERR,"[uORB] Failed to subscribe motor state data.\n");
		return ERROR;
	}

	/*
	 * 订阅PID控制参数的uORB消息
	 * 订阅消息失败,syslog报错记录,然后退出线程
	 * 订阅消息成功,保存订阅fd
	 */

	memset(&(sac->sub_pid_params->orb_data), 0, sizeof(sac->sub_pid_params->orb_data));

	sac->sub_pid_params->orb_fd = orb_subscribe(ORB_ID(pc_PID_cmd_uorb));

	if(sac->sub_pid_params->orb_fd < 0){
		syslog(LOG_ERR,"[uORB] Failed to subscribe pid parameters.\n");
		return ERROR;
	}

	/*
	 * 订阅目标姿态的uORB消息
	 * 清除变量内存
	 * 订阅消息失败,syslog报错记录,然后退出线程
	 * 订阅消息成功,保存订阅fd
	 */
	memset(&(sac->sub_attitude_sp->orb_data), 0, sizeof(sac->sub_attitude_sp->orb_data));

	sac->sub_attitude_sp->orb_fd = orb_subscribe(ORB_ID(attitude_set_point_uorb));

	if(sac->sub_attitude_sp->orb_fd < 0){
		syslog(LOG_ERR,"[uORB] Failed to subscribe attitude set point topic.\n");
		return ERROR;
	}

	/*
	 * 订阅卫星信息的uORB消息
	 * 清除变量内存
	 * 订阅消息失败,syslog报错记录,然后退出线程
	 * 订阅消息成功,保存订阅fd
	 */
	memset(&(sac->sub_sat_params->orb_data), 0, sizeof(sac->sub_sat_params->orb_data));

	sac->sub_sat_params->orb_fd = orb_subscribe(ORB_ID(pc_satellite_cmd_uorb));

	if(sac->sub_sat_params->orb_fd < 0){
		syslog(LOG_ERR,"[uORB] Failed to subscribe satellite params topic.\n");
		return ERROR;
	}


	return OK;
}
