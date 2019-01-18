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

#include <satellite_attitude_control/message_handle.h>
#include <satellite_attitude_control/satellite_attitude_control.h>

/****************************************************************************
 * Name: do_orb_msg_pre_check
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
int do_orb_msg_pre_check(void *priv)
{
	struct sat_att_ctrl_t  *sac = priv;
	bool update = false;
	static int log_tick_per_cycle = 20;								/* 用于控制syslog打印周期 */

	/* IMU */
	/* 如果使能该消息的检查位(enable_check == true) */
	if(sac->sub_attitude->enable_check){
		do
		{
			/* 查询该消息主题是否更新 */
			orb_check(sac->sub_attitude->orb_fd,&update);

			if(update){

				/* 拷贝最新的数据 */
				orb_copy(ORB_ID(imu_data_uorb), sac->sub_attitude->orb_fd, &(sac->sub_attitude->orb_data));

				/* 数据有效性检查 */
				if(fabs(sac->sub_attitude->orb_data.fusion_roll) <= 180.00 && \
				   fabs(sac->sub_attitude->orb_data.fusion_pitch)<= 180.00 && \
				   fabs(sac->sub_attitude->orb_data.fusion_yaw)  <= 180.00){

					/* pass!data is valid. */
					sac->sub_attitude->pass_check = true;

				}else{
					syslog(LOG_ERR,"[SAC] ERROR:attitude data is illegal.\n");
					return ERROR;
				}
			}
			/* wait for 50ms */
			usleep(50000);

			if(log_tick_per_cycle-- < 0){
				syslog(LOG_WARNING,"[SAC] WARN: Waiting attitude data...\n");
				log_tick_per_cycle = 20;  /* 20 * 50ms = 1s */
			}

		}while(!update && !sac->sub_attitude->pass_check);		/* pass, subscribe data updated and that is valid. */
	}

	/* motor feed-back */
	/* 如果使能该消息的检查位(enable_check == true) */
	if(sac->sub_motors_fb->enable_check){
		do
		{
			/* 查询该消息主题是否更新 */
			orb_check(sac->sub_motors_fb->orb_fd,&update);

			if(update){

				/* 拷贝最新的数据 */
				orb_copy(ORB_ID(motor_state_uorb), sac->sub_motors_fb->orb_fd, &(sac->sub_motors_fb->orb_data));

				/* 电机位置消息接收 */
				if(sac->sub_motors_fb->orb_data.mask == (0XFF >> (8-CONFIG_MOTOR_USER_NO)) ){

					/* 复制出最新数据 */
					sac->sub_motors_fb->position[0] = sac->sub_motors_fb->orb_data.params[0].fdata;
					sac->sub_motors_fb->position[1] = sac->sub_motors_fb->orb_data.params[1].fdata;
					sac->sub_motors_fb->position[2] = sac->sub_motors_fb->orb_data.params[2].fdata;

					/* 数据有效性检查 */
					if(   fabsf(sac->sub_motors_fb->orb_data.params[0].fdata) > 360\
						||fabsf(sac->sub_motors_fb->orb_data.params[1].fdata) > 360\
						||fabsf(sac->sub_motors_fb->orb_data.params[2].fdata) > 360){
						syslog(LOG_ERR,"[SAC]ERROR:motors feedback data is illegal.\n");
						return ERROR;
					}

					/* 检查通过 */
					sac->sub_motors_fb->pass_check = true;

					/* 清除电机的消息 */
					memset(&(sac->sub_motors_fb->orb_data), 0, sizeof(sac->sub_motors_fb->orb_data));
				}
			}
			/* wait for 50ms */
			usleep(50000);

			if(log_tick_per_cycle-- < 0){
				syslog(LOG_WARNING,"[SAC] WARN: Waiting motor state data...\n");
				log_tick_per_cycle = 20;  /* 20 * 50ms = 1s */
			}

		}while(!update && !sac->sub_motors_fb->pass_check);					/* pass, subscribe data updated and that is valid. */
	}
	return OK;
}
