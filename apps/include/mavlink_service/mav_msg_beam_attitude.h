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

#ifndef __APPS_MAVLINK_SERVICE_MSG_BEAM_ATTITUDE_H
#define __APPS_MAVLINK_SERVICE_MSG_BEAM_ATTITUDE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "uORB/topic/beam_attitude_uorb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
/*
 * attitude
 */
/*
 * 射束姿态
 */
struct msg_beam_attitude_t
{
	int 							fd;					/* mavlink message file descriptor */
	mavlink_ahrs2_t		 			msg;				/* motor state mavlink message */
	struct beam_attitude_uorb_s 	data;				/* motor state data */
}g_msg_beam_attitude;									/* 射束空间姿态 */
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mavlink_subscribe_beam_attitude
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
int mavlink_subscribe_beam_attitude(void *priv){

	struct msg_beam_attitude_t	*beam_attitude = &g_msg_beam_attitude;

	memset(&(beam_attitude->data), 0, sizeof(beam_attitude->data));
	beam_attitude->fd = orb_subscribe(ORB_ID(beam_attitude_uorb));

	return beam_attitude->fd;
}

/****************************************************************************
 * Name: mavlink_update_beam_attitude
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
int mavlink_update_beam_attitude(void *priv, hrt_abstime *abscurr)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_beam_attitude_t	*beam_attitude = &g_msg_beam_attitude;
	bool update = false;

	/* 查看当前是否有新的数据产生 */
	orb_check(beam_attitude->fd,&update);

	/* 产生新的数据 */
	if(update)
	{
		/* 拷贝最新鲜的数据 */
		orb_copy(ORB_ID(beam_attitude_uorb), beam_attitude->fd, &(beam_attitude->data));

		beam_attitude->msg.roll		= beam_attitude->data.roll;
		beam_attitude->msg.pitch	= beam_attitude->data.pitch;
		beam_attitude->msg.yaw		= beam_attitude->data.yaw;
		beam_attitude->msg.altitude	= beam_attitude->data.val1;
		beam_attitude->msg.lat		= beam_attitude->data.val2;
		beam_attitude->msg.lng		= beam_attitude->data.val3;


		//MAVLINK 组包
		mavlink_msg_ahrs2_pack(0x01, 0x01, &(mav->packet),
								beam_attitude->msg.roll,\
								beam_attitude->msg.pitch,\
								beam_attitude->msg.yaw,\
								beam_attitude->msg.altitude,\
								beam_attitude->msg.lat,\
								beam_attitude->msg.lng);

		//MAVLINK 拷贝完整数据包到待发送buff
		int len = mavlink_msg_to_send_buffer(mav->buf, &(mav->packet));
		if(len < 0 ){
			syslog(LOG_ERR, "[MAV]: attitude send error:%d\n",len);
		}

		/* 调用发送接口 发送MAVLINK message数据 */
		int ret = send_message_ethernet(mav,mav->buf,len,0);
		if(ret < 0 ){
			syslog(LOG_ERR, "[MAV]: send imu data error:%d\n",ret);
		}
	}

	return OK;
}

#define MAV_MSG_BEAM_ATTITUDE {"beam attitude",mavlink_subscribe_beam_attitude,mavlink_update_beam_attitude}

#endif /* __APPS_MAVLINK_SERVICE_MSG_BEAM_ATTITUDE_H */
