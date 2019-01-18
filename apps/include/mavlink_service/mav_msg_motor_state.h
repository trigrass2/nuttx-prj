/****************************************************************************
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_MAVLINK_SERVICE_MSG_MOTOR_STATE_H
#define __APPS_MAVLINK_SERVICE_MSG_MOTOR_STATE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

extern float test_dis_cmd[6];
extern float test_dis_posi[6];

#include <nuttx/config.h>
#include "uORB/topic/motor_state_uorb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
/*
 * attitude
 */
struct msg_motor_state_t
{
	int 							fd;					/* mavlink message file descriptor */
	mavlink_local_position_ned_t 	msg;				/* motor state mavlink message */
	struct motor_state_uorb_s 		data;				/* motor state data */
}g_msg_motor_state;										/* 电机反馈数据 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mavlink_subscribe_motor_state
 *
 * Description:
 *   subscribe heartbeat orb message.
 *
 * Input Parameters:
 *   priv  	- pointer of message.
 *
 * Returned Value:
 *   statue of subscribe.
 *
 ****************************************************************************/
int mavlink_subscribe_motor_state(void *priv){

	struct msg_motor_state_t	 *motor_state = &g_msg_motor_state;

	memset(&(motor_state->data), 0, sizeof(motor_state->data));
	motor_state->fd = orb_subscribe(ORB_ID(motor_state_uorb));

	return motor_state->fd;
}

/****************************************************************************
 * Name: mavlink_update_motor_state
 *
 * Description:
 *   update mavlink message.
 *
 * Input Parameters:
 *   priv  	- pointer of message.
 *
 * Returned Value:
 *   the size of message.
 *
 ****************************************************************************/
int mavlink_update_motor_state(void *priv, hrt_abstime *abscurr)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_motor_state_t *motor_state = &g_msg_motor_state;

	static hrt_abstime last;
	bool update = false;

	/* 查看当前是否有新的数据产生 */
	orb_check(motor_state->fd,&update);

	/* 产生新的数据 */
	if(update)
	{
		/* 拷贝最新鲜的数据 */
		orb_copy(ORB_ID(motor_state_uorb), motor_state->fd, &(motor_state->data));

		if(motor_state->data.mask == (0XFF >> (8-CONFIG_MOTOR_USER_NO))){

			/* 拷贝最新的数据 */
			if(motor_state->data.cmd[0] == 101)
				motor_state->msg.x  = motor_state->data.params[0].fdata; /*< [deg] Rx L1 Position*/
			if(motor_state->data.cmd[1] == 101)
				motor_state->msg.y  = motor_state->data.params[1].fdata; /*< [deg] Rx L2 Position*/
			if(motor_state->data.cmd[2] == 101)
				motor_state->msg.z  = motor_state->data.params[2].fdata; /*< [deg] Rx L3 Position*/

			motor_state->msg.time_boot_ms = *abscurr - last;

			//MAVLINK 组包
			mavlink_msg_local_position_ned_pack(0x01, 0x01, &(mav->packet),
												motor_state->msg.time_boot_ms,\
												motor_state->msg.x,\
												motor_state->msg.y,\
												motor_state->msg.z,\
												motor_state->msg.vx,\
												motor_state->msg.vy,\
												motor_state->msg.vz);


			//MAVLINK 拷贝完整数据包到待发送buff
			int len = mavlink_msg_to_send_buffer(mav->buf, &(mav->packet));
			if(len < 0 ){
				syslog(LOG_ERR, "[MAV]: motor state send error:%d\n",len);
			}

			/* 调用发送接口 发送MAVLINK message数据 */
			int ret = send_message_ethernet(mav,mav->buf,len,0);
			if(ret < 0 ){
				syslog(LOG_ERR, "[MAV]: send  motor state data error:%d\n",ret);
			}

			/* 保存上一次调用的绝对时间 */
			last = *abscurr;
		 }
	}

	return OK;
}

#define MAV_MSG_MOTOR_STATE {"motor state",mavlink_subscribe_motor_state,mavlink_update_motor_state}
#endif /* __APPS_MAVLINK_SERVICE_MSG_MOTOR_STATE_H */
