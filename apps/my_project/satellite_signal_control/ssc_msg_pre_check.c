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
 * Name: ssc_do_orb_msg_pre_check
 *
 * Description:
 *   pre-check the subscribed message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
int ssc_do_orb_msg_pre_check(void *priv)
{
	struct sat_signal_ctrl_t  *ssc = priv;
	bool update = false;
	static int log_tick_per_cycle = 20;								/* 用于控制syslog打印周期 */

	/* 卫星信标 */
	/* 如果使能该消息的检查位(enable_check == true) */
	if(ssc->sub_beacon_info->enable_check){
		do
		{
			/* 查询该消息主题是否更新 */
			orb_check(ssc->sub_beacon_info->orb_fd,&update);

			if(update){

				/* 拷贝最新的数据 */
				orb_copy(ORB_ID(beacon_data_uorb), ssc->sub_beacon_info->orb_fd, &(ssc->sub_beacon_info->orb_data));

				/* 数据有效性检查 */
				if(fabsf(ssc->sub_beacon_info->orb_data.beacon_signal.power) <= 130.00 && \
				   fabsf(ssc->sub_beacon_info->orb_data.beacon_signal.voltage)<= 130.00){

					/* pass!data is valid. */
					ssc->sub_beacon_info->pass_check = true;

				}else{
					syslog(LOG_ERR,"[SSC] ERROR:satellite beacon data is illegal.\n");
					return ERROR;
				}
			}
			/* wait for 50ms */
			usleep(50000);

			if(log_tick_per_cycle-- < 0){
				syslog(LOG_WARNING,"[SSC] WARN: Waiting satellite beacon data...\n");
				log_tick_per_cycle = 20;  /* 20 * 50ms = 1s */
			}

		}while(!update && !ssc->sub_beacon_info->pass_check);		/* pass, subscribe data updated and that is valid. */
	}

	/* 卫星位置 */
	/* 如果使能该消息的检查位(enable_check == true) */
	if(ssc->sub_satellite_info->enable_check){
		do
		{
			/* 查询该消息主题是否更新 */
			orb_check(ssc->sub_satellite_info->orb_fd,&update);

			if(update){

				/* 拷贝最新的数据 */
				orb_copy(ORB_ID(satellite_point_uorb), ssc->sub_satellite_info->orb_fd, &(ssc->sub_satellite_info->orb_data));

				/* 数据有效性检查 */
				if(fabsf(ssc->sub_satellite_info->orb_data.fusion_pitch) <= 90.00 && \
				   fabsf(ssc->sub_satellite_info->orb_data.fusion_yaw)<= 180.00){

					/* pass!data is valid. */
					ssc->sub_satellite_info->pass_check = true;

				}else{
					syslog(LOG_ERR,"[SSC] ERROR:satellite point data is illegal.\n");
					return ERROR;
				}
			}
			/* wait for 50ms */
			usleep(50000);

			if(log_tick_per_cycle-- < 0){
				syslog(LOG_WARNING,"[SSC] WARN: Waiting satellite point data...\n");
				log_tick_per_cycle = 20;  /* 20 * 50ms = 1s */
			}

		}while(!update && !ssc->sub_satellite_info->pass_check);		/* pass, subscribe data updated and that is valid. */
	}

	return OK;
}
