/****************************************************************************
 * app/include/satellite_signal_control/satellite_signal_control.h
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

#ifndef __APPS_SATELLITE_SIGNAL_CONTROL_H
#define __APPS_SATELLITE_SIGNAL_CONTROL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <poll.h>
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MODE_CASE(x)			(x/10)

/****************************************************************************
 * Public Types
 ****************************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/
enum sat_att_ctrl_mode
{
	INIT 	= 0,										/* 初始化[0~9] */
	INIT_MSG_AND_PARAMS,								/* 初始化:找零 */
	INIT_WAIT_ATTITUDE_COMPLETED,						/* 初始化:等待找零完成 */
	INIT_WAIT_COMMAND,									/* 初始化:找零完成,等待指令 */
	RUN 	= 10,										/* 执行[10~19] */
	RUN_POSITION_SEARCH,								/* 执行:目标位置搜索 */
	RUN_DISTURBANCE_SEARCH_YAW,							/* 执行:随机扰动搜索 */
	RUN_DISTURBANCE_SEARCH_PITCH,						/* 执行:随机扰动搜索 */
	RUN_SIGNAL_SEARCH,									/* 执行:信号收敛搜索 */
	STOP 	= 20,										/* 停止[20~29] */
	IDLE 	= 30,										/* 空闲[30~39] */
};

/* 目标卫星值 */
struct target_satellite_t
{
	struct postion_t{
		float 	roll;									/* 横滚设定值 有效值[-90,90] */
		float 	pitch;									/* 俯仰设定值 有效值[-90,90] */
		float 	yaw;									/* 横滚设定值 有效值[-180,180] */
		float 	last_roll;								/* 横滚设定值 有效值[-90,90] */
		float 	last_pitch;								/* 俯仰设定值 有效值[-90,90] */
		float 	last_yaw;								/* 横滚设定值 有效值[-180,180] */
	}postion;

};

/* 卫星信标数据 */
struct beacon_info_t
{
	bool	lock;										/* lock true:已锁定  false:未锁定 */
	bool	effective;									/* 有效性 */
	float	power;										/* 信标功率值 */
	float	voltage;									/* 信标电压值 */
};

struct sat_signal_ctrl_t
{
	/* main task parameters */
	pid_t 						task_pid;				/* task pid */
	int 						mode;					/* antenna run-time mode */
	bool 						should_exit;			/* task loop exit flag */
	int  						pre_check_list;			/* message pre-check list */
	struct pollfd 				fds;					/* file descriptor of poll */
	float fcut;
	int duty;

	/* 目标卫星 */
	struct target_satellite_t 				*target_satellite;

	/*controller parameters */
	struct spsa_algorithm_t					*spsa_yaw;
	struct spsa_algorithm_t					*spsa_pitch;

	/* subscribed data */
	struct orb_satellite_info_t				*sub_satellite_info;			/* 卫星位置信息 */
	struct orb_beacon_info_t 				*sub_beacon_info;				/* 卫星信标消息  */
	struct orb_attitude_control_status_t 	*sub_attitude_control_status;	/* 卫星目标消息  */

	/* advertise data */
	struct orb_attitude_set_point_t  		*pub_attitude_sp;				/* 姿态设置目标消息 */
};
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __APPS_SATELLITE_SIGNAL_CONTROL_H */
