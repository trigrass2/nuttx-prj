/****************************************************************************
 *  apps/my_project/satellite_attitude_control/mode_stop.c
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
#include <satellite_attitude_control/satellite_attitude_control.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: mode_stop_run
 *
 * Description:
 *   satellite attitude control stop.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int mode_stop_run(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;
	//1 关闭PID<俯仰使能><航向使能>
	sac->rxatt_pitch->enable = false;
	sac->rxatt_yaw->enable = false;
	sac->rxatt_yaw2->enable = false;

	//2 清空PID过程变量
	sac->rxatt_pitch->last_derivative = 0.0;
	sac->rxatt_pitch->integrator = 0.0;
	sac->rxatt_pitch->last_error = 0.0;
	memset(&(sac->rxatt_pitch->info),0,sizeof(sac->rxatt_pitch->info));

	sac->rxatt_yaw->last_derivative = 0.0;
	sac->rxatt_yaw->integrator = 0.0;
	sac->rxatt_yaw->last_error = 0.0;
	memset(&(sac->rxatt_yaw->info),0,sizeof(sac->rxatt_yaw->info));

	sac->rxatt_yaw2->last_derivative = 0.0;
	sac->rxatt_yaw2->integrator = 0.0;
	sac->rxatt_yaw2->last_error = 0.0;
	memset(&(sac->rxatt_yaw2->info),0,sizeof(sac->rxatt_yaw2->info));

	//3 取消相关消息订阅

	//4 退出任务
	sac->should_exit = true;

	return OK;
}

