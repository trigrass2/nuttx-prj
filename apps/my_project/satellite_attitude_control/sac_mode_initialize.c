/****************************************************************************
 *  apps/my_project/satellite_attitude_control/mode_initialize.c
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

#include <satellite_attitude_control/pid.h>
#include <satellite_attitude_control/params_storage.h>
#include <satellite_attitude_control/message_handle.h>
#include <satellite_attitude_control/satellite_attitude_control.h>

#include <pc_control/pc_control.h>

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
 * Name: mode_initizlize_run
 *
 * Description:
 *   satellite attitude control initialize.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int mode_initizlize_run(void *psat_att_ctrl)
{

	struct sat_att_ctrl_t *sac = psat_att_ctrl;
	int ret = 0;
	switch(sac->mode){

		/*
		 * 初始化INITIALIZE
		 * 1 复位天线
		 * 	1.1 L1、L2、L3位置找零
		 * 	1.2 找零完成，绑定航向大地坐标系角度
		 */
		case INIT_SEARCH_HOME:
		{
			/*
			 * 电机ID就是cmd数组的下标[0,5],目前找零是写任意一个电机0x03 其余盘都会找零
			 * 0x03:电机找零(设置任意一个电机,所有盘都会找零)
			 * 0x04:设置绝对位置[0.0,360.0]
			 * 0x0C:设置相对位置[-1080.0,1080.0]
			 * 0x14:设置速度[-100.0,100.0]
			 */
			memset(&(sac->pub_motors_cmd->msg),0,sizeof(sac->pub_motors_cmd->msg));

			/* 电机找零 */
			sac->pub_motors_cmd->msg.cmd[0] = 0x03;

			ret = orb_publish(ORB_ID(pc_motor_cmd_uorb), sac->pub_motors_cmd->pub, &(sac->pub_motors_cmd->msg));
			if(ret < 0){

				syslog(LOG_ERR,"[SAC] failed to publish pc_motor_cmd_uorb topic :%d.\n",sac->pub_motors_cmd->pub);

				return ERROR;
			}else{

				/* 标记进入等待找零完成 */
				syslog(LOG_INFO,"[SAC]waiting motors search home completed.\n");

				/* change state machine to INIT_WAIT_COMPLETE */
				sac->mode = INIT_WAIT_COMPLETE;
			}
			
		}break;


		/*
		 * 等待初始化找零完成
		 */
		case INIT_WAIT_COMPLETE:
		{
			if((sac->sub_motors_fb->orb_data.state[0] == MOTOR_CMD_STATE_COMPLETE) && (sac->sub_motors_fb->orb_data.cmd[0] == 0x03)){

				/* 标记位置找零完成 */
				syslog(LOG_INFO,"[SAC]motors search home completed.\n");

				/* 清除数据 */
				memset(&(sac->sub_motors_fb->orb_data),0,sizeof(sac->sub_motors_fb->orb_data));

				/* change state machine to INIT_WAIT_COMPLETE */
				sac->mode = INIT_WAIT_COMMAND;
			}
		}break;

		case INIT_WAIT_COMMAND:
		{
			/* 所有状态正常，进入姿态控制模式 */
			syslog(LOG_INFO,"[SAC]motors initialize completed.\n");
			syslog(LOG_INFO,"[SAC]enter mode RUN_ATTITUDE_CONTROL.\n");
			/* there is no task to do,just run "run" */
			sac->mode = RUN_ATTITUDE_CONTROL;

		}break;
	}
	return OK;
}


