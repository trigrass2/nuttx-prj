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

#ifndef __APPS_MAVLINK_SERVICE_H
#define __APPS_MAVLINK_SERVICE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

enum
{
	IMU_RAW,
	MOTOR_STATE,
	ATTITUDE,
	MAX
};


/****************************************************************************
 * Public Types
 ****************************************************************************/
struct mavlink_parse_t
{

	bool initialized;									/* hardware initialize true:initialized  false:un-initialize */
	pid_t task_pid;
	bool should_exit;

	//tcp/ip
	int sockfd;											/* socket fd */
	int client_fd;										/* client fd */
	struct sockaddr_in server_addr;						/* server address */

	struct sockaddr_in udp_client;
	struct sockaddr_in udp_client_sub;

	//uorb
	bool subscribe;										/* subscribe initialize true:subscribed  false:un-subscribe */
	struct pollfd fds[MAX];								/* array of poll fd */
	int fd[MAX];										/* array of mavlink message fd */

	uint8_t 					buf[256];
	mavlink_message_t  			packet;

};

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: send_message_ethernet
 *
 * Description:
 *   send mavlink message
 *
 * Input Parameters:
 *   pointer  - pointer of message.
 *
 * Returned Value:
 *   the size of message.
 *
 ****************************************************************************/
extern inline int send_message_ethernet(void *priv,void *buff,int len,int flog);


#endif /* __APPS_MAVLINK_SERVICE_H */
