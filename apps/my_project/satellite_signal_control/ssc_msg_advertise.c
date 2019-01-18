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

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <debug.h>
#include <errno.h>

#include <satellite_signal_control/ssc_msg_handle.h>
#include <satellite_signal_control/satellite_signal_control.h>

/****************************************************************************
 * Name: ssc_do_orb_msg_advertise
 *
 * Description:
 *   advertise the uorb message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
int ssc_do_orb_msg_advertise(void *priv)
{
	struct sat_signal_ctrl_t  *ssc = priv;

	memset(&(ssc->pub_attitude_sp->msg), 0, sizeof(ssc->pub_attitude_sp->msg));

	/*
	 * 登记姿态设定位置消息
	 */
	ssc->pub_attitude_sp->pub = orb_advertise(ORB_ID(attitude_set_point_uorb), &(ssc->pub_attitude_sp->msg));

	/* 登记失败报错退出 */
	if(ssc->pub_attitude_sp->pub == NULL){
		syslog(LOG_ERR,"[SAC] failed to advertise attitude set point topic :%d.\n",ssc->pub_attitude_sp->pub);
		return ERROR;
	}else{
		syslog(LOG_INFO,"[SAC] advertised attitude set point topic.\n");
	}

	return OK;
}


