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

#include <satellite_signal_control/ssc_msg_handle.h>
#include <satellite_signal_control/satellite_signal_control.h>

/****************************************************************************
 * Name: ssc_do_orb_msg_poll
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

int ssc_do_orb_msg_poll(void * priv)
{
	struct sat_signal_ctrl_t  *ssc = priv;

	/* poll all subscribed uorb message */
	int poll_ret = poll(&(ssc->fds), 1, 500);

	/* 处理poll结果 */
	if (poll_ret < 0) {

		/* 错误异常,必须处理 */
		syslog(LOG_ERR,"[SSC]ERROR:failed to poll data: %d\n", poll_ret);
	}
	/* 超时异常 */
	else if(poll_ret == 0){

		syslog(LOG_ERR,"[SSC]ERROR:timeout to poll data: %d\n", poll_ret);
	}
	/* 检查需要更新的数据,执行对应逻辑 */
	else{

		bool update = false;

		/*
		 * 检查姿态控制信息
		 */
		orb_check(ssc->sub_attitude_control_status->orb_fd,&update);
		if(update){

			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(attitude_control_status_uorb),ssc->sub_attitude_control_status->orb_fd, &(ssc->sub_attitude_control_status->orb_data));

		}

		/*
		 * 检查卫星信标信息
		 */
		orb_check(ssc->sub_beacon_info->orb_fd,&update);
		if(update){

			static float power_last = 0.0;
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(beacon_data_uorb),ssc->sub_beacon_info->orb_fd, &(ssc->sub_beacon_info->orb_data));

//	        float RC = 1/(2 * M_PI * ssc->fcut);
//	        ssc->sub_beacon_info->orb_data.beacon_signal.power = power_last +( (50000 / (RC + 50000)) *(ssc->sub_beacon_info->orb_data.beacon_signal.power - power_last) );
//	        power_last = ssc->sub_beacon_info->orb_data.beacon_signal.power;
		}

		/*
		 * 检查卫星位置信息
		 */
		orb_check(ssc->sub_satellite_info->orb_fd,&update);
		if(update){

			static float power_last = 0.0;

			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(satellite_point_uorb),ssc->sub_satellite_info->orb_fd, &(ssc->sub_satellite_info->orb_data));

			if(ssc->sub_satellite_info->orb_data.fusion_pitch < 20 || fabsf(ssc->sub_satellite_info->orb_data.fusion_yaw) > 180){
				return -1;
			}else{
				/* 检查数据合法性 */
				if(ssc->sub_satellite_info->orb_data.fusion_pitch > 20){
					ssc->target_satellite->postion.pitch = ssc->sub_satellite_info->orb_data.fusion_pitch;
				}else{
					ssc->target_satellite->postion.pitch = ssc->sub_satellite_info->orb_data.fusion_pitch = 20;
				}
				ssc->target_satellite->postion.yaw = ssc->sub_satellite_info->orb_data.fusion_yaw;
			}
		}

	}

	return poll_ret;
}
