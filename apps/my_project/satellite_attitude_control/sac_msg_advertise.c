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
 * Name: ssc_do_orb_msg_advertise
 *
 * Description:
 *   advertise the uorb message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
int do_orb_msg_advertise(void *priv)
{
	struct sat_att_ctrl_t  *sac = priv;
	//L1/L2/L3控制输出

	memset(&(sac->pub_motors_cmd->msg), 0, sizeof(sac->pub_motors_cmd->msg));

	/*
	 * 登记电机控制命令主题
	 */
	sac->pub_motors_cmd->pub = orb_advertise(ORB_ID(pc_motor_cmd_uorb), &(sac->pub_motors_cmd->msg));

	/* 登记失败报错退出 */
	if(sac->pub_motors_cmd->pub == NULL){
		syslog(LOG_ERR,"[SAC] failed to advertise pc_motor_cmd_uorb topic :%d.\n",sac->pub_motors_cmd->pub);
		return ERROR;
	}else{
		syslog(LOG_INFO,"[SAC] advertised pc_motor_cmd_uorb topic.\n");
	}

	/*
	 *  登记射束姿态主题
	 */
	sac->pub_beam_attitude->pub = orb_advertise(ORB_ID(beam_attitude_uorb), &(sac->pub_beam_attitude->msg));

	/* 登记失败报错退出 */
	if(sac->pub_beam_attitude->pub == NULL){
		syslog(LOG_ERR,"[SAC] failed to advertise beam_attitude topic :%d.\n",sac->pub_beam_attitude->pub);
		return ERROR;
	}else{
		syslog(LOG_INFO,"[SAC] advertised beam_attitude topic.\n");
	}

	/*
	 * 登记PID整定消息
	 */
	sac->pub_pid_turning->pub = orb_advertise(ORB_ID(pid_turning_data_uorb), &(sac->pub_pid_turning->msg));

	/* 登记失败报错退出 */
	if(sac->pub_pid_turning->pub == NULL){
		syslog(LOG_ERR,"[SAC] failed to advertise pid_turning topic :%d.\n",sac->pub_pid_turning->pub);
		return ERROR;
	}else{
		syslog(LOG_INFO,"[SAC] advertised pid_turning topic.\n");
	}

	/*
	 * 登记PID参数消息
	 */
	sac->pub_pid_params_fb->pub = orb_advertise(ORB_ID(pc_PID_params_uorb), &(sac->pub_pid_params_fb->msg));

	/* 登记失败报错退出 */
	if(sac->pub_pid_params_fb->pub == NULL){
		syslog(LOG_ERR,"[SAC] failed to advertise pid params topic :%d.\n",sac->pub_pid_params_fb->pub);
		return ERROR;
	}else{
		syslog(LOG_INFO,"[SAC] advertised  pid params topic.\n");
	}

	/*
	 * 登记姿态控制状态消息
	 */
	sac->pub_attitude_control_status->pub = orb_advertise(ORB_ID(attitude_control_status_uorb), &(sac->pub_attitude_control_status->msg));

	/* 登记失败报错退出 */
	if(sac->pub_attitude_control_status->pub == NULL){
		syslog(LOG_ERR,"[SAC] failed to advertise attitude control status topic :%d.\n",sac->pub_attitude_control_status->pub);
		return ERROR;
	}else{
		syslog(LOG_INFO,"[SAC] advertised attitude control status topic.\n");
	}

	return OK;
}


