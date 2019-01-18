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

#ifndef __APPS_MAVLINK_SERVICE_MSG_IMU_DATA_H
#define __APPS_MAVLINK_SERVICE_MSG_IMU_DATA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "uORB/topic/imu_data_uorb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
/*
 * attitude
 */
struct msg_attitude_t
{
	int 							fd;					/* mavlink message file descriptor */
	mavlink_attitude_t 				msg;				/* raw imu mavlink message */
	struct imu_data_uorb_s 			data;;				/* combined imu data */
}g_msg_attitude;										/* 融合后载体姿态 */
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: mavlink_subscribe_attitude
 *
 * Description:
 *   subscribe the orb message.
 *
 * Input Parameters:
 *   priv  	- pointer of message.
 *
 * Returned Value:
 *   statue of subscribe.
 *
 ****************************************************************************/
int mavlink_subscribe_attitude(void *priv)
{
	struct msg_attitude_t *attitude	= &g_msg_attitude;

	memset(&(attitude->data), 0, sizeof((attitude->data)));
	attitude->fd = orb_subscribe(ORB_ID(imu_data_uorb));

	return attitude->fd;
}
/****************************************************************************
 * Name: mavlink_update_attitude
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
int mavlink_update_attitude(void *priv, hrt_abstime *abscurr)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_attitude_t *attitude	= &g_msg_attitude;

	static hrt_abstime last;
	bool update = false;

	/* 查看当前是否有新的数据产生 */
	orb_check(attitude->fd,&update);

	/* 产生新的数据 */
	if(update)
	{
		/* 拷贝最新鲜的数据 */
		orb_copy(ORB_ID(imu_data_uorb), attitude->fd, &(attitude->data));

		attitude->msg.roll		 	= attitude->data.fusion_roll;
		attitude->msg.pitch			= attitude->data.fusion_pitch;
		attitude->msg.yaw			= attitude->data.fusion_yaw;
		attitude->msg.rollspeed		= attitude->data.gx;
		attitude->msg.pitchspeed	= attitude->data.gy;
		attitude->msg.yawspeed	 	= attitude->data.gz;
		attitude->msg.time_boot_ms = *abscurr - last;

		//MAVLINK 组包
		mavlink_msg_attitude_pack(0x01, 0x01, &(mav->packet),
								attitude->msg.time_boot_ms,\
								attitude->msg.roll,\
								attitude->msg.pitch,\
								attitude->msg.yaw,\
								attitude->msg.rollspeed,\
								attitude->msg.pitchspeed,\
								attitude->msg.yawspeed);

		//MAVLINK 拷贝完整数据包到待发送buff
		int len = mavlink_msg_to_send_buffer(mav->buf, &(mav->packet));

		/* 调用发送接口 发送MAVLINK message数据 */
		int ret = send_message_ethernet(mav,mav->buf,len,0);
		if(ret < 0 ){
			syslog(LOG_ERR, "[MAV]: send attitude error:%d\n",ret);
		}

		/* 保存上一次调用的绝对时间 */
		last = *abscurr;
	}

	return OK;
}

#define MAV_MSG_ATTITUDE {"body attitude",mavlink_subscribe_attitude,mavlink_update_attitude}

#endif /* __APPS_MAVLINK_SERVICE_MSG_IMU_DATA_H */
