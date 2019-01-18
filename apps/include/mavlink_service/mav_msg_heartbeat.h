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

#ifndef __APPS_MAVLINK_SERVICE_MSG_HEARTBEAT_H
#define __APPS_MAVLINK_SERVICE_MSG_HEARTBEAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
/*
 * heart beat
 */
struct msg_heartbeat_t
{
	int 							fd;					/* mavlink message file descriptor */
	mavlink_heartbeat_t 			msg;				/* heart beat mavlink message */
	hrt_abstime 					last_time;			/* absolute time to record last call */

}g_msg_heartbeat;
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mavlink_subscribe_heartbeat
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
int mavlink_subscribe_heartbeat(void *priv){

	return 0;
}


/****************************************************************************
 * Name: mavlink_update_heartbeat
 *
 * Description:
 *   packet and send mavlink message.
 *
 * Input Parameters:
 *   priv  	- pointer of message.
 *
 * Returned Value:
 *   the size of message.
 *
 ****************************************************************************/
int mavlink_update_heartbeat(void *priv, hrt_abstime *abscurr)
{
	struct mavlink_parse_t * mav = priv;
	struct msg_heartbeat_t * heartbeat = &g_msg_heartbeat;

	if((*abscurr - heartbeat->last_time) < 1000000)return 0;

	heartbeat->last_time = *abscurr;
	//heartbeat
	mavlink_msg_heartbeat_pack(0x01, 0x01, &(mav->packet),
								heartbeat->msg.type,\
								heartbeat->msg.autopilot,\
								heartbeat->msg.base_mode,\
								heartbeat->msg.custom_mode,\
								heartbeat->msg.system_status);

	int len = mavlink_msg_to_send_buffer(mav->buf, &(mav->packet));

	int ret = send_message_ethernet(mav,mav->buf,len,0);

	return ret;
}


#define MAV_MSG_HEARTBEAT {"heartbeat", mavlink_subscribe_heartbeat, mavlink_update_heartbeat}

#endif /* __APPS_MAVLINK_SERVICE_MSG_HEARTBEAT_H */
