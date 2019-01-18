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

#ifndef __APPS_MAVLINK_SERVICE_MSG_BEACON_POWER_H
#define __APPS_MAVLINK_SERVICE_MSG_BEACON_POWER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "uORB/topic/beacon_data_uorb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct msg_beacon_power_t
{
	int 							fd;					/* uorb file descriptor */
	mavlink_beacon_power_t 			msg;				/* mavlink message */
	struct beacon_data_uorb_s 		data;				/* uorb data */
}g_msg_beacon_power;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mavlink_subscribe_beacon_power
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
int mavlink_subscribe_beacon_power(void *priv)
{
	struct msg_beacon_power_t *beacon_power	= &g_msg_beacon_power;

	memset(&(beacon_power->data), 0, sizeof(beacon_power->data));
	beacon_power->fd = orb_subscribe(ORB_ID(beacon_data_uorb));

	return beacon_power->fd;
}

/****************************************************************************
 * Name: mavlink_update_beacon_power
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
int mavlink_update_beacon_power(void *priv, hrt_abstime *abscurr)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_beacon_power_t *beacon_power	= &g_msg_beacon_power;
	bool update = false;

	/* 查看当前是否有新的数据产生 */
	orb_check(beacon_power->fd,&update);

	/* 产生新的数据 */
	if(update)
	{
		/* 拷贝最新鲜的数据 */
		orb_copy(ORB_ID(beacon_data_uorb), beacon_power->fd, &(beacon_power->data));

		beacon_power->msg.power		= beacon_power->data.beacon_signal.power;
		beacon_power->msg.isLock	= beacon_power->data.beacon_signal.lock;
		beacon_power->msg.isValid	= beacon_power->data.beacon_signal.effective;


		//MAVLINK 组包
		mavlink_msg_beacon_power_pack(0x01, 0x01, &(mav->packet),
								beacon_power->msg.power,\
								beacon_power->msg.isLock,\
								beacon_power->msg.isValid);

		//MAVLINK 拷贝完整数据包到待发送buff
		int len = mavlink_msg_to_send_buffer(mav->buf, &(mav->packet));

		/* 调用发送接口 发送MAVLINK message数据 */
		int ret = send_message_ethernet(mav,mav->buf,len,0);
		if(ret < 0 ){
			syslog(LOG_ERR, "[MAV]: send beacon power data error:%d\n",ret);
		}
	}

	return OK;
}


#define MAV_MSG_BEACON_POWER {"beacon power",mavlink_subscribe_beacon_power,mavlink_update_beacon_power}


#endif /* __APPS_MAVLINK_SERVICE_MSG_BEACON_POWER_H */
