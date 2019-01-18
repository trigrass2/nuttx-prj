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

#include <satellite_signal_control/ssc_msg_handle.h>
#include <satellite_signal_control/satellite_signal_control.h>

/****************************************************************************
 * Name: ssc_do_orb_msg_subscribe
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
int ssc_do_orb_msg_subscribe(void *priv)
{
	struct sat_signal_ctrl_t  *ssc = priv;

	/*
	 * 订阅beacon的uORB消息
	 * 清除变量内存
	 * 订阅消息失败,syslog报错记录,然后退出线程
	 * 订阅消息成功,保存订阅fd
	 */
	memset(&(ssc->sub_beacon_info->orb_data), 0, sizeof(ssc->sub_beacon_info->orb_data));

	ssc->sub_beacon_info->orb_fd = orb_subscribe(ORB_ID(beacon_data_uorb));

	if(ssc->sub_beacon_info->orb_fd < 0){

		syslog(LOG_ERR,"[uORB] Failed to subscribe beacon data.\n");

		return ERROR;
	}

	/*
	 * 订阅姿态控制的uORB消息
	 * 清除变量内存
	 * 订阅消息失败,syslog报错记录,然后退出线程
	 * 订阅消息成功,保存订阅fd
	 */
	memset(&(ssc->sub_attitude_control_status->orb_data), 0, sizeof(ssc->sub_attitude_control_status->orb_data));

	ssc->sub_attitude_control_status->orb_fd = orb_subscribe(ORB_ID(attitude_control_status_uorb));

	if(ssc->sub_attitude_control_status->orb_fd < 0){

		syslog(LOG_ERR,"[uORB] Failed to subscribe attitude control status.\n");

		return ERROR;
	}

	/*
	 * 订阅卫星位置的uORB消息
	 * 清除变量内存
	 * 订阅消息失败,syslog报错记录,然后退出线程
	 * 订阅消息成功,保存订阅fd
	 */
	memset(&(ssc->sub_satellite_info->orb_data), 0, sizeof(ssc->sub_satellite_info->orb_data));

	ssc->sub_satellite_info->orb_fd = orb_subscribe(ORB_ID(satellite_point_uorb));

	if(ssc->sub_satellite_info->orb_fd < 0){

		syslog(LOG_ERR,"[uORB] Failed to subscribe satellite point uorb.\n");

		return ERROR;
	}

	/*
	 * 设定poll读取目标
	 */
	ssc->fds.fd 		= ssc->sub_attitude_control_status->orb_fd;
	ssc->fds.events 	= POLLIN;

	return OK;
}
