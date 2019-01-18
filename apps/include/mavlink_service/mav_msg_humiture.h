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

#ifndef __APPS_MAVLINK_SERVICE_MSG_HUMITURE_H
#define __APPS_MAVLINK_SERVICE_MSG_HUMITURE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "uORB/topic/temperature_data_uorb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
/*
 * attitude
 */
struct msg_temperature_data_t
{
	int 							fd;					/* uorb file descriptor */
	mavlink_humiture_t 				msg;				/* mavlink message */
	struct temperature_data_uorb_s	data;				/* uorb data */
}g_msg_temperature;										/* 温度湿度消息 */
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mavlink_subscribe_temperature_data
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
int mavlink_subscribe_temperature_data(void *priv)
{
	struct msg_temperature_data_t *temperature	= &g_msg_temperature;

	memset(&(temperature->data), 0, sizeof(temperature->data));
	temperature->fd = orb_subscribe(ORB_ID(temperature_data_uorb));

	return temperature->fd;
}

/****************************************************************************
 * Name: mavlink_update_temperature
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
int mavlink_update_temperature(void *priv, hrt_abstime *abscurr)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_temperature_data_t *temperature = &g_msg_temperature;
	bool update = false;

	/* 查看当前是否有新的数据产生 */
	orb_check(temperature->fd,&update);

	/* 产生新的数据 */
	if(update)
	{
		/* 拷贝最新鲜的数据 */
		orb_copy(ORB_ID(temperature_data_uorb),temperature->fd, &(temperature->data));

		temperature->msg.HUMIDITY   = temperature->data.humidity;
		temperature->msg.TEMPRETURE = temperature->data.temperature;

		//MAVLINK 组包
		mavlink_msg_humiture_pack(0x01, 0x01, &(mav->packet),
								temperature->msg.TEMPRETURE,\
								temperature->msg.HUMIDITY);

		//MAVLINK 拷贝完整数据包到待发送buff
		int len = mavlink_msg_to_send_buffer(mav->buf, &(mav->packet));

		/* 调用发送接口 发送MAVLINK message数据 */
		int ret = send_message_ethernet(mav,mav->buf,len,0);
		if(ret < 0 ){
			syslog(LOG_ERR, "[MAV]: send temperature data error:%d\n",ret);
		}
	}

	return OK;
}

#define MAV_MSG_TEMPERATURE {"temperature",mavlink_subscribe_temperature_data,mavlink_update_temperature}


#endif /* __APPS_MAVLINK_SERVICE_MSG_HUMITURE_H */
