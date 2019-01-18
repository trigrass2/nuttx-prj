/****************************************************************************
 *  apps/my_project/satellite_signal_control/mode_initialize.c
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
#include <nuttx/init.h>

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>

#include <debug.h>
#include <errno.h>

#include <satellite_signal_control/ssc_msg_handle.h>
#include <satellite_signal_control/satellite_signal_control.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Data
 *********************************************************s*******************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: ssc_mode_initizlize_run
 *
 * Description:
 *   satellite signal control initialize.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int ssc_mode_initizlize_run(void *psat_att_ctrl)
{

	struct sat_signal_ctrl_t *ssc = psat_att_ctrl;

	int ret = 0;

	switch(ssc->mode){

		/*
		 * 等待姿态找零完成
		 *
		 */
		case INIT_WAIT_ATTITUDE_COMPLETED:
		{
			/*
			 * 等待姿态进入控制模式
			 */
			if(ssc->sub_attitude_control_status->orb_data.mode >= 10 ){

				/* 标记等待姿态找零完成 */
				syslog(LOG_INFO,"[SSC]waiting attitude control completed.\n");

				/* change state machine to INIT_WAIT_COMPLETE */
				ssc->mode = INIT_WAIT_COMMAND;
			}
		}break;

		case INIT_WAIT_COMMAND:
		{
			/* 所有状态正常，进入搜索控制模式 */
			syslog(LOG_INFO,"[SSC]start searching target satellite position...\n");

			/* there is no task to do,just run "run" */
			ssc->mode = RUN_POSITION_SEARCH;

		}break;
	}
	return OK;
}


