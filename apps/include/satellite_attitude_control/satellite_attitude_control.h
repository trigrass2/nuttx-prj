/****************************************************************************
 * app/include/satellite_attitude_control/satellite_attitude_control.h
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

#ifndef __APPS_SATELLITE_ATTITUDE_CONTROL_H
#define __APPS_SATELLITE_ATTITUDE_CONTROL_H

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
	INIT_SEARCH_HOME,									/* 初始化:找零 */
	INIT_WAIT_COMPLETE,									/* 初始化:等待找零完成 */
	INIT_WAIT_COMMAND,									/* 初始化:找零完成,等待指令 */
	RUN 	= 10,										/* 执行[10~19] */
	RUN_ATTITUDE_CONTROL,								/* 执行：姿态控制 */
	STOP 	= 20,										/* 停止[20~29] */
	IDLE 	= 30,										/* 空闲[30~39] */
};

/* command to control antenna attitude */
struct attitude_cmd_t
{
	int 	mode;
	float 	ned_roll;
	float 	ned_pitch;
	float 	ned_yaw;
	float 	T;
};

struct attitude_info_rt_t
{
	sq_entry_t sq;
	float roll;
	float pitch;
	float yaw;
};

struct sat_att_ctrl_t
{
	/* main task parameters */
	pid_t 						task_pid;				/* task pid */
	int 						mode;					/* antenna run-time mode */
	bool 						should_exit;			/* task loop exit flag */
	int  						pre_check_list;			/* message pre-check list */
	struct pollfd 				fds;					/* file descriptor of poll */

	/*controller parameters */
	//PID
	struct algorithm_pid_t 		*rxatt_pitch;			/* received antenna pitch direction control parameters */
	struct algorithm_pid_t 		*rxatt_yaw;				/* received antenna yaw direction control parameters */
	struct algorithm_pid_t 		*rxatt_yaw2;			/* received antenna yaw direction control parameters */
	struct algorithm_pid_t 		*rxatt_polar;			/* received antenna yaw direction control parameters */
	struct algorithm_pid_t 		*rxatt_polar2;			/* received antenna yaw direction control parameters */
	struct algorithm_pid_t 		*rxatt_T;				/* received antenna T angle control parameters */
	//FUZZY_PID
	struct algorithm_fuzzy_pid_t 	*rx_pitch;			/* received antenna pitch direction control parameters */
	struct algorithm_fuzzy_pid_t 	*rx_yaw;			/* received antenna yaw direction control parameters */
	struct algorithm_fuzzy_pid_t 	*rx_polar;			/* received antenna polar direction control parameters */


	/* beam data */
	struct beam_attitude_t 		*beam;					/* antenna beam attitude data */
	struct attitude_info_rt_t   *attitude_sq;

	/* command to control antenna attitude */
	struct attitude_cmd_t 		*cmd;					/* command to control antenna attitude  */

	/* subscribed data */
	struct orb_attitude_t 		*sub_attitude;			/* sensor combine */
	struct orb_motors_fb_t 		*sub_motors_fb;			/* motor feed-back data */
	struct orb_pid_params_t 	*sub_pid_params;		/* pid parameters data */
	struct orb_satellite_params_t  *sub_sat_params;		/* 卫星参数 */
	struct orb_attitude_set_point_t *sub_attitude_sp;	/* attitude set point */

	/* advertise data */
	struct orb_motors_cmd_t 	*pub_motors_cmd;		/* motors command */
	struct orb_beam_attitude_t  *pub_beam_attitude;		/* beam attitude */
	struct orb_pid_turning_t	*pub_pid_turning;		/* pid_turning */
	struct orb_pid_params_fb_t	*pub_pid_params_fb;		/* pid params feed back */
	struct orb_attitude_control_status_t *pub_attitude_control_status;	/* attitude control status */
};
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __APPS_SATELLITE_ATTITUDE_CONTROL_H */
