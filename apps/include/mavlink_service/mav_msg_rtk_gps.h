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

#ifndef __APPS_MAVLINK_SERVICE_MSG_RTK_GPS_H
#define __APPS_MAVLINK_SERVICE_MSG_RTK_GPS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "uORB/topic/gps_data_uorb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
/*
 * attitude
 */
struct msg_gps_data_t
{
	int 							fd;					/* mavlink message file descriptor */
	mavlink_rtk_gps_t 				msg;				/* raw imu mavlink message */
	struct gps_data_uorb_s 			data;				/* combined rtk data */
}g_msg_gps_data;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mavlink_subscribe_rtk_gps
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
int mavlink_subscribe_rtk_gps(void *priv)
{
	struct msg_gps_data_t *gps	= &g_msg_gps_data;

	memset(&(gps->data), 0, sizeof(gps->data));
	gps->fd = orb_subscribe(ORB_ID(gps_data_uorb));

	return gps->fd;
}


/****************************************************************************
 * Name: mavlink_update_rtk_gps
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
int mavlink_update_rtk_gps(void *priv, hrt_abstime *abscurr)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_gps_data_t *gps	= &g_msg_gps_data;
	bool update = false;

	/* 查看当前是否有新的数据产生 */
	orb_check(gps->fd,&update);

	/* 产生新的数据 */
	if(update)
	{
		/* 拷贝最新鲜的数据 */
		orb_copy(ORB_ID(gps_data_uorb), gps->fd, &(gps->data));

		gps->msg.height	 			= gps->data.altitude;
		gps->msg.azimuth			= gps->data.azimuth;
		gps->msg.orientation		= gps->data.course;
		gps->msg.status    			= gps->data.gps_state;
		gps->msg.level_precision	= gps->data.hdop;
		gps->msg.vertical_precision	= gps->data.vdop;
		gps->msg.com_precision		= gps->data.pdop;
		gps->msg.latitude		 	= gps->data.latitude;
		gps->msg.longitude			= gps->data.longitude;
		gps->msg.number				= gps->data.satellite_no;
		gps->msg.speed	 			= gps->data.speed;
		gps->msg.year		 		= gps->data.time.year;
		gps->msg.month			 	= gps->data.time.month;
		gps->msg.day		 		= gps->data.time.day;
		gps->msg.hour		 		= gps->data.time.hour;
		gps->msg.minute		 		= gps->data.time.minutes;
		gps->msg.second		 		= gps->data.time.second;

		//MAVLINK 组包
		mavlink_msg_rtk_gps_pack(0x01, 0x01, &(mav->packet),
								gps->msg.status,\
								gps->msg.longitude,\
								gps->msg.latitude,\
								gps->msg.azimuth,\
								gps->msg.height,\
								gps->msg.speed,\
								gps->msg.orientation,\
								gps->msg.com_precision,\
								gps->msg.level_precision,\
								gps->msg.vertical_precision,\
								gps->msg.number,\
								gps->msg.year,\
								gps->msg.month,\
								gps->msg.day,\
								gps->msg.hour,\
								gps->msg.minute,\
								gps->msg.second);

		//MAVLINK 拷贝完整数据包到待发送buff
		int len = mavlink_msg_to_send_buffer(mav->buf, &(mav->packet));

		/* 调用发送接口 发送MAVLINK message数据 */
		int ret = send_message_ethernet(mav,mav->buf,len,0);
		if(ret < 0 ){
			syslog(LOG_ERR, "[MAV]: send rtk gps error:%d\n",ret);
		}
	}

	return OK;
}

#define MAV_MSG_RTK_GPS {"rtk gps",mavlink_subscribe_rtk_gps,mavlink_update_rtk_gps}

#endif /* __APPS_MAVLINK_SERVICE_MSG_RTK_GPS_H */
