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

#ifndef __APPS_MAVLINK_SERVICE_MSG_RAW_IMU_H
#define __APPS_MAVLINK_SERVICE_MSG_RAW_IMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "uORB/topic/adis16488_uorb.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
struct msg_raw_imu_t
{
	int 							fd;					/* mavlink message file descriptor */
	mavlink_raw_imu_t 				msg;				/* raw imu mavlink message */
	struct adis16488_uorb_s 		adis16488;			/* adis16488 raw data */
}g_msg_raw_imu;											/* 原始IMU数据:adis16488 */
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mavlink_subscribe_raw_imu
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
int mavlink_subscribe_raw_imu(void *priv)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_raw_imu_t * raw_imu = &g_msg_raw_imu;

	memset(&(raw_imu->adis16488), 0, sizeof(raw_imu->adis16488));
	raw_imu->fd = orb_subscribe(ORB_ID(adis16488_uorb));

	mav->fds[IMU_RAW].fd = raw_imu->fd;
	mav->fds[IMU_RAW].events 	= POLLIN;

	return raw_imu->fd;
}

/****************************************************************************
 * Name: mavlink_update_raw_imu
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
int mavlink_update_raw_imu(void *priv, hrt_abstime *abscurr)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_raw_imu_t * raw_imu = &g_msg_raw_imu;
	static hrt_abstime last;
	bool update = false;

	/* 查看当前是否有新的数据产生 */
	orb_check(raw_imu->fd,&update);

	/* 产生新的数据 */
	if(update)
	{
		/* 拷贝最新鲜的数据 */
		orb_copy(ORB_ID(adis16488_uorb), raw_imu->fd, &(raw_imu->adis16488));

		raw_imu->msg.xacc		= raw_imu->adis16488.acc[0];
		raw_imu->msg.yacc		= raw_imu->adis16488.acc[1];
		raw_imu->msg.zacc		= raw_imu->adis16488.acc[2];
		raw_imu->msg.xgyro	 	= raw_imu->adis16488.gyro[0];
		raw_imu->msg.ygyro	 	= raw_imu->adis16488.gyro[1];
		raw_imu->msg.zgyro	 	= raw_imu->adis16488.gyro[2];
		raw_imu->msg.xmag		= raw_imu->adis16488.mag[0];
		raw_imu->msg.ymag		= raw_imu->adis16488.mag[1];
		raw_imu->msg.zmag		= raw_imu->adis16488.mag[2];
		raw_imu->msg.time_usec = *abscurr - last;

		//MAVLINK 组包
		mavlink_msg_raw_imu_pack(0x01, 0x01, &(mav->packet),\
								raw_imu->msg.time_usec,\
								raw_imu->msg.xacc,\
								raw_imu->msg.yacc,\
								raw_imu->msg.zacc,\
								raw_imu->msg.xgyro,\
								raw_imu->msg.ygyro,\
								raw_imu->msg.zgyro,\
								raw_imu->msg.xmag,\
								raw_imu->msg.ymag,\
								raw_imu->msg.zmag);

		//MAVLINK 拷贝完整数据包到待发送buff
		int len = mavlink_msg_to_send_buffer(mav->buf, &(mav->packet));

		/* 调用发送接口 发送MAVLINK message数据 */
		int ret = send_message_ethernet(mav,mav->buf,len,0);
		if(ret < 0 ){
			syslog(LOG_ERR, "[MAV]: raw imu send error:%d\n",ret);
		}

		/* 保存上一次调用的绝对时间 */
		last = *abscurr;
	}

	return OK;
}

#define MAV_MSG_RAW_IMU {"raw imu",mavlink_subscribe_raw_imu,mavlink_update_raw_imu}

#endif /* __APPS_MAVLINK_SERVICE_MSG_RAW_IMU_H */
