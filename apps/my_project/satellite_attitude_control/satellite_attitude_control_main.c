/****************************************************************************
 *  apps/my_project/satellite_attitude_control/satellite_attitude_control_main.c
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
#include <nuttx/timers/drv_hrt.h>

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <queue.h>

#include <debug.h>
#include <errno.h>

#include <satellite_attitude_control/pid.h>
#include <satellite_attitude_control/fuzzy_pid.h>
#include <satellite_attitude_control/params_storage.h>
#include <satellite_attitude_control/message_handle.h>
#include <satellite_attitude_control/sac_mode_initialize.h>
#include <satellite_attitude_control/sac_mode_run.h>
#include <satellite_attitude_control/sac_mode_stop.h>
#include <satellite_attitude_control/sac_beam_convert.h>
#include <satellite_attitude_control/satellite_attitude_control.h>
#include <satellite_attitude_control/adrc.h>
#include <DSP_Lib/arm_math.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Data
 ****************************************************************************/
/*
 * subscribed data about attitude by inertia sensor combine
 */
struct orb_attitude_t g_orb_attitude_primal ={
	.pass_check 	= false,
	.enable_check	= true,
	.orb_fd			= 0,
};

/*
 * subscribed data about motor feed-back
 */
struct orb_motors_fb_t g_orb_motor_fb ={
	.pass_check 	= false,
	.enable_check	= true,
	.orb_fd			= 0,
	.position[0]	= 0.0,
	.position[1]	= 0.0,
	.position[2]	= 0.0,
};

/*
 * advertise data about motor command
 */
struct orb_motors_cmd_t 			 g_orb_motors_cmd;
struct orb_beam_attitude_t			 g_orb_beam_attitude_cmd;
struct orb_pid_turning_t			 g_orb_pid_turning;
struct orb_pid_params_t				 g_orb_pid_params;
struct orb_pid_params_fb_t			 g_orb_pid_params_fb;
struct orb_attitude_set_point_t	     g_orb_attitude_sp;
struct orb_attitude_control_status_t g_orb_attitude_control_status;
struct orb_satellite_params_t		 g_orb_satellite_params;
/*
 * PID Controller Parameters For Received Antenna In Pitch Direction
 */
struct algorithm_pid_t g_rxatt_pitch_pid = {
	/* used for PID calculate and store */
	.params =
	{
		.kp = 0.9,					/* proportional */
		.ki = 1.0,					/* integral */
		.kd = 0.2,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 25,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integer */
		.o_max = 50.00,				/* maxim value of PID controller */

	},

	/* record the value during PID calculating */
	.info =
	{
		.P = 0.0,					/* result of proportional */
		.I = 0.0,					/* result of integral */
		.D = 0.0,					/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	.integrator = 0.0,
	.last_error = 0.0,
	.last_derivative = 0.0,

	.axis_id = 0x10,
	.adrc_params = &ADRC_Pitch_Controller,
	.td_params = &TD_Pitch_Controller,
	.iir_filter_aFilt = &pitch_aFilt[0],
	.iir_filter_bFilt = &pitch_bFilt[0],

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */

};

/*
 * Controller Parameters For Received Antenna In Yaw Direction
 */
struct algorithm_pid_t g_rxatt_yaw_pid ={
	/* used for PID calculate and store */
	.params =
	{
		.kp = 1.0,					/* proportional */
		.ki = 1.0,					/* integral */
		.kd = 0.2,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 40,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integer */
		.o_max = 50.00,				/* maxim value of PID controller */

	},

	/* record the value during PID calculating */
	.info ={
		.P 		 = 0.0,				/* result of proportional */
		.I 		 = 0.0,				/* result of integral */
		.D 		 = 0.0,				/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	.integrator 	 = 0.0,
	.last_error 	 = 0.0,
	.last_derivative = 0.0,

	.axis_id = 0x20,
	.adrc_params = &ADRC_yaw_Controller,
	.td_params = &TD_yaw_Controller,
	.iir_filter_aFilt = &yaw_aFilt[0],
	.iir_filter_bFilt = &yaw_bFilt[0],	

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */
};

/*
 * Control Parameters For Received Antenna In Yaw Direction
 */
struct algorithm_pid_t g_rxatt_yaw2_pid ={
	/* used for PID calculate and store */
	.params =
	{
		.kp = 0.8,					/* proportional */
		.ki = 0.4,					/* integral */
		.kd = 0.08,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 40,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integer */
		.o_max = 50.00,				/* maxim value of PID controller */

	},

	/* record the value during PID calculating */
	.info ={
		.P 		 = 0.0,				/* result of proportional */
		.I 		 = 0.0,				/* result of integral */
		.D 		 = 0.0,				/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	.integrator 	 = 0.0,
	.last_error 	 = 0.0,
	.last_derivative = 0.0,

	.axis_id = 0x21,
	.adrc_params = &ADRC_yaw_Controller,
	.td_params = &TD_yaw_Controller,
	.iir_filter_aFilt = &yaw_aFilt[0],
	.iir_filter_bFilt = &yaw_bFilt[0],		

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */
};

/*
 * Control Parameters For Received Antenna In polar Direction
 */
struct algorithm_pid_t g_rxatt_polar_pid ={
	/* used for PID calculate and store */
	.params =
	{
		.kp = 1.0,					/* proportional */
		.ki = 1.0,					/* integral */
		.kd = 0.2,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 40,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integer */
		.o_max = 50.00,				/* maxim value of PID controller */

	},

	/* record the value during PID calculating */
	.info ={
		.P 		 = 0.0,				/* result of proportional */
		.I 		 = 0.0,				/* result of integral */
		.D 		 = 0.0,				/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	.integrator 	 = 0.0,
	.last_error 	 = 0.0,
	.last_derivative = 0.0,

	.axis_id 			= 0x30,
	.adrc_params 		= &ADRC_polar_Controller,
	.td_params 			= &TD_polar_Controller,
	.iir_filter_aFilt 	= &polar_aFilt[0],
	.iir_filter_bFilt 	= &polar_bFilt[0],

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */
};

/*
 * Control Parameters For Received Antenna In Yaw Direction
 */
struct algorithm_pid_t g_rxatt_polar2_pid ={
	/* used for PID calculate and store */
	.params =
	{
		.kp = 0.8,					/* proportional */
		.ki = 0.4,					/* integral */
		.kd = 0.05,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 40,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integer */
		.o_max = 50.00,				/* maxim value of PID controller */

	},

	/* record the value during PID calculating */
	.info ={
		.P 		 = 0.0,				/* result of proportional */
		.I 		 = 0.0,				/* result of integral */
		.D 		 = 0.0,				/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	.integrator 	 = 0.0,
	.last_error 	 = 0.0,
	.last_derivative = 0.0,

	.axis_id = 0x31,
	.adrc_params = &ADRC_polar_Controller,
	.td_params = &TD_polar_Controller,
	.iir_filter_aFilt = &polar_aFilt[0],
	.iir_filter_bFilt = &polar_bFilt[0],	

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */
};

/*
 * Control Parameters For Received Antenna In Yaw Direction
 */
struct algorithm_pid_t g_rxatt_T_pid ={
	/* used for PID calculate and store */
	.params =
	{
		.kp = 0.8,					/* proportional */
		.ki = 0.4,					/* integral */
		.kd = 0.05,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 40,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integer */
		.o_max = 50.00,				/* maxim value of PID controller */

	},

	/* record the value during PID calculating */
	.info ={
		.P 		 = 0.0,				/* result of proportional */
		.I 		 = 0.0,				/* result of integral */
		.D 		 = 0.0,				/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	.integrator 	 = 0.0,
	.last_error 	 = 0.0,
	.last_derivative = 0.0,

	.axis_id = 0x40,
	.adrc_params = NULL,
	.td_params = NULL,
	.iir_filter_aFilt = NULL,
	.iir_filter_bFilt = NULL,		

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */
};

/*
 *fuzzy PID Controller Parameters For Received Antenna In Yaw Direction
 */
struct algorithm_fuzzy_pid_t g_rx_yaw_fuzzy_pid ={
	/* used for PID calculate and store */
	.params =
	{
		.kp = 1.6,					/* proportional */
		.ki = 0.4,					/* integral */
		.kd = 0.05,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 40,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integer */
		.o_max = 50.00,				/* maxim value of PID controller */
		.i_threshold = 1,			/* threshold of integral seperate */

	},

	/* record the value during PID calculating */
	.info ={
		.P 		 = 0.0,				/* result of proportional */
		.I 		 = 0.0,				/* result of integral */
		.D 		 = 0.0,				/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	/* 模糊控制表 */
	.tab	={
			 7,									/* 偏差列:7 */
			 3,									/* 微分行:3 */
			{ {NB ,NB ,NS},						/* 比例表 */
			  {NB ,NM ,NS},
			  {NM ,NM ,ZO},
			  {NS ,ZO ,PS},
			  {ZO ,PM ,PM},
			  {PS ,PM ,PB},
			  {PS ,PB ,PB} },
			{ {3 ,4 ,3}, },						/* 积分表 */
			{ {NB ,NM ,ZO},						/* 微分表 */
			  {NB ,NM ,ZO},
			  {NM ,NS ,ZO},
			  {NS ,ZO ,PS},
			  {ZO ,PS ,PM},
			  {ZO ,PM ,PB},
			  {ZO ,PM ,PB}  },
           {-0.3,-0.15,-0.07,0,0.07,0.15,0.3,},	/* 偏差段 */
           {-3,0,3,},							/* 微分段 */
	},

	.integrator 	 = 0.0,
	.last_error 	 = 0.0,
	.last_derivative = 0.0,

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */
};

/*
 *fuzzy PID Controller Parameters For Received Antenna In Pitch Direction
 */
struct algorithm_fuzzy_pid_t g_rx_pitch_fuzzy_pid ={
	/* used for PID calculate and store */
	.params =
	{
		.kp = 1.8,					/* proportional */
		.ki = 0.4,					/* integral */
		.kd = 0.05,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 40,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integral */
		.o_max = 50.00,				/* maxim value of PID controller */
		.i_threshold = 1,			/* threshold of integral seperate */
	},

	/* record the value during PID calculating */
	.info ={
		.P 		 = 0.0,				/* result of proportional */
		.I 		 = 0.0,				/* result of integral */
		.D 		 = 0.0,				/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	/* 模糊控制表 */
	.tab	={
			 7,									/* 偏差列:7 */
			 3,									/* 微分行:3 */
			{ {NB ,NB ,NS},						/* 比例表 */
			  {NB ,NM ,NS},
			  {NM ,NM ,ZO},
			  {NS ,ZO ,PS},
			  {ZO ,PM ,PM},
			  {PS ,PM ,PB},
			  {PS ,PB ,PB} },
			{ {3 ,4 ,3}, },						/* 积分表 */
			{ {NB ,NM ,ZO},						/* 微分表 */
			  {NB ,NM ,ZO},
			  {NM ,NS ,ZO},
			  {NS ,ZO ,PS},
			  {ZO ,PS ,PM},
			  {ZO ,PM ,PB},
			  {ZO ,PM ,PB}  },
           {-0.3,-0.15,-0.07,0,0.07,0.15,0.3,},	/* 偏差段 */
           {-3,0,3,},							/* 微分段 */
	},

	.integrator 	 = 0.0,
	.last_error 	 = 0.0,
	.last_derivative = 0.0,

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */
};

/*
 *fuzzy PID Controller Parameters For Received Antenna In Polar Direction
 */
struct algorithm_fuzzy_pid_t g_rx_polar_fuzzy_pid ={
	/* used for PID calculate and store */
	.params =
	{
		.kp = 1.6,					/* proportional */
		.ki = 0.4,					/* integral */
		.kd = 0.05,					/* derivative */
		.kf = 0.0,					/* feedforward */
		.fCut = 40,					/* used for input filter. unit: hz */
		.i_max = 30.00,				/* maxim value of integer */
		.o_max = 50.00,				/* maxim value of PID controller */
		.i_threshold = 1,			/* threshold of integral seperate */

	},

	/* record the value during PID calculating */
	.info ={
		.P 		 = 0.0,				/* result of proportional */
		.I 		 = 0.0,				/* result of integral */
		.D 		 = 0.0,				/* result of derivative */
		.desired = 0.0				/* result of PID controller */
	},

	/* 模糊控制表 */
	.tab	={
			 7,									/* 偏差列:7 */
			 3,									/* 微分行:3 */
			{ {NB ,NB ,NS},						/* 比例表 */
			  {NB ,NM ,NS},
			  {NM ,NM ,ZO},
			  {NS ,ZO ,PS},
			  {ZO ,PM ,PM},
			  {PS ,PM ,PB},
			  {PS ,PB ,PB} },
			{ {3 ,4 ,3}, },						/* 积分表 */
			{ {NB ,NM ,ZO},						/* 微分表 */
			  {NB ,NM ,ZO},
			  {NM ,NS ,ZO},
			  {NS ,ZO ,PS},
			  {ZO ,PS ,PM},
			  {ZO ,PM ,PB},
			  {ZO ,PM ,PB}  },
           {-0.3,-0.15,-0.07,0,0.07,0.15,0.3,},	/* 偏差段 */
           {-3,0,3,},							/* 微分段 */
	},

	.integrator 	 = 0.0,
	.last_error 	 = 0.0,
	.last_derivative = 0.0,

	.last_t = 0,					/* unit: microsecond */
	.enable = true,					/* switch of PID controller. true:enable  false:disable */
};

struct attitude_cmd_t g_attitude_cmd = {
		.ned_roll 	= 0.0,        	/* 设置目标射束的极化角度 */
		.ned_pitch 	= 45,			/* 设置目标射束的俯仰角度 */
		.ned_yaw 	= 0,	   	    /* 设置目标射束的航向角度 */
};

/*
 * 控制状态队列
 */
struct attitude_info_rt_t g_attitude_info_rt = {
	.roll 	= 0.0,
	.pitch 	= 0.0,
	.yaw 	= 0.0,
};

/*
 * antenna beam attitude
 */
#define FREQ_18_7GHz (1)
#define FREQ_19_8GHz (2)

struct beam_attitude_t g_beam =
{
	.params ={

			/* ROLL */
			/*19.8GHz*/
			.L 	= -2.01191,
			.A1 =  2.17749,
			.A2 = -0.54394,
			.A3 =  0.05127,
			.A4 = -0.00214,
			.A5 =  4.08337e-5,
			.A6 = -2.85197e-7,
			/*18.7GHz*/
//			.L 	= -5.41387,
//			.A1 =  2.32646,
//			.A2 = -0.50218,
//			.A3 =  0.0474,
//			.A4 = -0.00205,
//			.A5 =  4.07329e-5,
//			.A6 = -2.97495e-7,
			/*全部拟合*/
//			.L 	= -1.57851,
//			.A1 =  2.06372,
//			.A2 = -0.47044,
//			.A3 =  0.04385,
//			.A4 = -0.00186,
//			.A5 =  3.59955e-5,
//			.A6 = -2.56008e-7,

			/* PITCH */
			/*19.8GHz*/
			.M 	=  4.60812,
			.B1 =  0.16926,
			.B2 =  0.09039,
			.B3 = -0.00347,
			.B4 =  5.27811e-5,
			.B5 =  1.65685e-9,
			.B6 = -3.88465e-9,
			/*18.7GHz*/
//			.M 	=  9.52775,
//			.B1 =  0.18847,
//			.B2 =  0.03427,
//			.B3 =  0.00167,
//			.B4 = -1.46783e-4,
//			.B5 =  3.65739e-6,
//			.B6 = -2.94452e-8,
			/*全部拟合*/
//			.M 	=  6.20553,
//			.B1 =  0.249,
//			.B2 =  0.05467,
//			.B3 = -3.33634e-4,
//			.B4 = -6.7511e-5,
//			.B5 =  2.15815e-6,
//			.B6 = -1.85408e-8,

			/* YAW */
			/*19.8GHz*/
			.N 	=  173.62772,
			.C1 =  14.57336,
			.C2 = -1.17466,
			.C3 =  0.04956,
			.C4 = -0.00115,
			.C5 =  1.36817e-5,
			.C6 = -6.55156e-8,
//			/*18.7GHz*/
//			.N 	=  175.69815,
//			.C1 =  7.09301,
//			.C2 = -0.20792,
//			.C3 = -0.00519,
//			.C4 =  4.34147e-4,
//			.C5 = -9.06009e-6,
//			.C6 =  6.35507e-8,
			/*全部拟合*/
//			.N 	=  176.27333,
//			.C1 =  11.8705,
//			.C2 = -0.86532,
//			.C3 =  0.03379,
//			.C4 = -7.34239e-4,
//			.C5 =  8.24683e-6,
//			.C6 = -3.70718e-8,
	},
	.attitude ={
			.T 			= 0.0,							/* T角:以L2盘为基准,L1盘逆时针旋转为正[有效范围0~50度] */
			.BASE		= 0.0,							/* B角:以L2盘零位为基准,逆时针旋转为正[有效范围0~360度] */
			.t_roll 	= 0.0,							/* 射束横滚角(未叠加姿态):以X轴为旋转轴顺时针为正[有效范围-45~45] */
			.t_pitch 	= 0.0,							/* 射束俯仰角(未叠加姿态):以Y轴为旋转轴顺时针为正[有效范围0~50] */
			.t_yaw   	= 0.0,							/* 射束偏航角(未叠加姿态):以Z轴为旋转轴顺时针为正[有效范围-180~180] */
			.ned_roll	= 0.0,							/* 射束横滚角:以X轴为旋转轴顺时针为正[有效范围-180~180] */
			.ned_pitch	= 0.0,							/* 射束横滚角:以Y轴为旋转轴顺时针为正[有效范围-180~180] */
			.ned_yaw 	= 0.0,							/* 射束横滚角:以Z轴为旋转轴顺时针为正[有效范围-180~180] */
	}
};

/*
 * All Control Parameters For Received Antenna
 */
static struct sat_att_ctrl_t  g_sat_att_ctrl ={
	.task_pid 			= 0,							/* 任务PID */
	.mode 				= STOP,							/* 姿态控制模式:详见模式枚举 */
	.should_exit 		= true,							/* 任务退出flog */
	.cmd				= &g_attitude_cmd,				/* 姿态控制命令 */
	//线性PID
	.rxatt_pitch 		= &g_rxatt_pitch_pid,			/* 俯仰控制 */
	.rxatt_yaw 			= &g_rxatt_yaw_pid,				/* 航向控制 */
	.rxatt_yaw2 		= &g_rxatt_yaw2_pid,			/* 航向控制 */
	.rxatt_polar 		= &g_rxatt_polar_pid,			/* 极化控制 */
	.rxatt_polar2 		= &g_rxatt_polar2_pid,			/* 极化控制2 */
	.rxatt_T			= &g_rxatt_T_pid,				/* T控制 */
	//模糊PID
	.rx_polar			= &g_rx_polar_fuzzy_pid,		/* 模糊极化控制 */
	.rx_pitch			= &g_rx_pitch_fuzzy_pid,		/* 模糊俯仰控制 */
	.rx_yaw				= &g_rx_yaw_fuzzy_pid,			/* 模糊航向控制 */
	//波束
	.beam				= &g_beam,						/* 射束状态 */

	//
	.attitude_sq		= &g_attitude_info_rt,
	//orb
	.sub_attitude 		= &g_orb_attitude_primal,		/* orb订阅姿态消息 */
	.sub_motors_fb		= &g_orb_motor_fb,				/* orb订阅电机状态消息 */
	.sub_attitude_sp    = &g_orb_attitude_sp,			/* orb订阅姿态目标数据 */
	.sub_pid_params		= &g_orb_pid_params,			/* orb订阅PID控制消息 */
	.sub_sat_params		= &g_orb_satellite_params,		/* orb订阅PID控制消息 */
	.pub_motors_cmd		= &g_orb_motors_cmd,			/* orb发布电机控制消息 */
	.pub_beam_attitude	= &g_orb_beam_attitude_cmd,		/* orb发布射束状态消息 */
	.pub_pid_params_fb	= &g_orb_pid_params_fb,			/* orb发布PID参数消息 */
	.pub_pid_turning	= &g_orb_pid_turning,			/* PID调试信息 */
	.pub_attitude_control_status = &g_orb_attitude_control_status,/* orb发布控制状态 */

};


/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: sat_att_ctrl_stable_decision
 *
 * Description:
 *   satellite attitude control decision.
 *
 * Input Parameters:
 *   argc  - number of parameters.
 *   argv  - pointer of parameters.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
//bool sat_att_ctrl_stable_decision(void *priv)
//{
//	struct sat_att_ctrl_t  *sac = priv;
//
//	if(){
//		return true;
//	}
//	else{
//		return false;
//	}
//}


/****************************************************************************
 * Name: sat_att_ctrl_run
 *
 * Description:
 *   satellite attitude control runtime.
 *
 * Input Parameters:
 *   argc  - number of parameters.
 *   argv  - pointer of parameters.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int sat_att_ctrl_run(int argc, FAR char *argv[])
{

	/***************************************************************************
	 * [参数读取]
	 * 从存储器中读取姿态控制参数
	 **************************************************************************/

	/*
	 * 从EEPROM中读取俯仰方向的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	int nbytes = 0;
	nbytes = params_load(&(g_rxatt_pitch_pid.params), ADDR_ATT_RX_CTRL_PITCH, sizeof(struct pid_params_t));
	if(nbytes != sizeof(struct pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[pitch].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[pitch][%d bytes].\n",nbytes);
	}

	/*
	 * 从EEPROM中读取偏航方向的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	nbytes = params_load(&(g_rxatt_yaw_pid.params), ADDR_ATT_RX_CTRL_YAW, sizeof(struct pid_params_t));
	if(nbytes != sizeof(struct pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[yaw].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[yaw][%d bytes].\n",nbytes);
	}

	/*
	 * 从EEPROM中读取偏航2方向的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	nbytes = params_load(&(g_rxatt_yaw2_pid.params), ADDR_ATT_RX_CTRL_YAW2, sizeof(struct pid_params_t));
	if(nbytes != sizeof(struct pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[yaw2].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[yaw2][%d bytes].\n",nbytes);
	}

	/*
	 * 从EEPROM中读取极化方向的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	nbytes = params_load(&(g_rxatt_polar_pid.params), ADDR_ATT_RX_CTRL_POLAR, sizeof(struct pid_params_t));
	if(nbytes != sizeof(struct pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[polar].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[polar][%d bytes].\n",nbytes);
	}

	/*
	 * 从EEPROM中读取极化2方向的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	nbytes = params_load(&(g_rxatt_polar2_pid.params), ADDR_ATT_RX_CTRL_POLAR2, sizeof(struct pid_params_t));
	if(nbytes != sizeof(struct pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[polar2].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[polar2][%d bytes].\n",nbytes);
	}

	/*
	 * 从EEPROM中读取T的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	nbytes = params_load(&(g_rxatt_T_pid.params), ADDR_ATT_RX_CTRL_T, sizeof(struct pid_params_t));
	if(nbytes != sizeof(struct pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[T].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[T][%d bytes].\n",nbytes);
	}

	/*
	 * 从EEPROM中读取FUZZY极化的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	nbytes = params_load(&(g_rx_polar_fuzzy_pid.params), ADDR_ATT_RX_CTRL_FUZZY_POLAR, sizeof(struct fuzzy_pid_params_t));
	if(nbytes != sizeof(struct fuzzy_pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[FUZZY_POLAR].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[FUZZY_POLAR][%d bytes].\n",nbytes);
	}

	/*
	 * 从EEPROM中读取FUZZY航向的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	nbytes = params_load(&(g_rx_yaw_fuzzy_pid.params), ADDR_ATT_RX_CTRL_FUZZY_YAW, sizeof(struct fuzzy_pid_params_t));
	if(nbytes != sizeof(struct fuzzy_pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[FUZZY_YAW].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[FUZZY_YAW][%d bytes].\n",nbytes);
	}

	/*
	 * 从EEPROM中读取FUZZY俯仰的控制参数
	 * 读取失败,syslog报错,然后退出线程
	 * 读取成功,syslog记录读取信息
	 */
	nbytes = params_load(&(g_rx_pitch_fuzzy_pid.params), ADDR_ATT_RX_CTRL_FUZZY_PITCH, sizeof(struct fuzzy_pid_params_t));
	if(nbytes != sizeof(struct fuzzy_pid_params_t)){
		syslog(LOG_ERR,"[SAC] Failed to load received antenna attitude control parameters[FUZZY_PITCH].\n");
		return -1;
	}else{
		syslog(LOG_INFO,"[SAC] Loaded received antenna attitude control parameters[FUZZY_PITCH][%d bytes].\n",nbytes);
	}


	/****************************************************************************
	 * [消息订阅]
	 * 订阅相关uORB消息
	 ****************************************************************************/
	int ret = 0;
	ret = do_orb_msg_subscribe(&g_sat_att_ctrl);
	if(ret < 0){
		syslog(LOG_ERR,"[SAC] Failed to subscribe orb message:%d\n",ret);
		return ret;
	}else{
		syslog(LOG_INFO,"[SAC] Subscribed orb message\n");
	}
	/*****************************************************************************
	 * [消息检查]
	 * 消息订阅检查,进入循环前,检查各个消息状态是否正常(是否正常接收\数据是否正常)
	 *****************************************************************************/
	ret = do_orb_msg_pre_check(&g_sat_att_ctrl);
	if(ret < 0){
		syslog(LOG_ERR,"[SAC] Failed to pre-check message:%d\n",ret);
		return ret;
	}else{
		syslog(LOG_INFO,"[SAC] Pass to check orb message\n");
	}

	/******************************************************************************
	 * [消息登记]
	 * 发布电机控制量数据(发布的周期由控制周期决定)
	 ******************************************************************************/
	ret = do_orb_msg_advertise(&g_sat_att_ctrl);
	if(ret < 0){
		syslog(LOG_ERR,"[SAC] Failed to advertised orb message:%d\n",ret);
		return ret;
	}else{
		syslog(LOG_INFO,"[SAC] Advertised orb message\n");
	}
	
	/******************************************************************************
	 * [ADRC数据初始化]
	 * 
	 ******************************************************************************/
	ADRC_Init(g_rxatt_pitch_pid.adrc_params,&adrc_pitch_par[0]);
	ADRC_Init(g_rxatt_yaw_pid.adrc_params,&adrc_yaw_par[0]);
	ADRC_Init(g_rxatt_polar_pid.adrc_params,&adrc_polar_par[0]);

	td_Init(g_rxatt_pitch_pid.td_params,&td_pitch_par[0][0]);
	td_Init(g_rxatt_yaw_pid.td_params,&td_yaw_par[0][0]);
	td_Init(g_rxatt_polar_pid.td_params,&td_polar_par[0][0]);
	/******************************************************************************
	 * [数据处理]
	 * 1.数据更新:
	 * 			1) uORB订阅电机位置/IMU姿态数据/目标位置信息;
	 * 2.模式处理:
	 * 			1) INIT[初始化模式]:实现位置找零,等待进入[解算模式];
	 * 			2) Run[解算模式]:依据目标位置/IMU姿态信息进行PID运算;
	 * 			3) Stop[停止模式]:停止姿态解算,关闭相关数据;
	 *
	 ******************************************************************************/

	/* 默认初始找零 */
	g_sat_att_ctrl.mode = INIT_SEARCH_HOME;
//	 static hrt_abstime last;
	 while(!g_sat_att_ctrl.should_exit){

//		 hrt_abstime abscurr = hrt_absolute_time();
//		 int t = abscurr - last;
//		 printf("cycle:%d \n",t);
		/*
		 * 1 数据更新
		 *  1.1 poll UORB数据更新，超时设定100ms
		 *  1.2 timeout异常处理
		 *  1.3 poll失败异常处理
		 */
		 ret = do_orb_msg_poll(&g_sat_att_ctrl);
		 if(ret <= 0){
			 syslog(LOG_ERR,"ERROR:failed to poll message : %d\n", ret);
		 }
		/*
		 * 2 数据处理
		 *  2.1 模式选择
		 *  2.2 执行对应模式逻辑
		 */
		switch(MODE_CASE(g_sat_att_ctrl.mode)){

			/*
			 * 初始化INITIALIZE
			 * 1 复位天线
			 * 	1.1 L1、L2、L3位置找零
			 * 	1.2 找零完成，绑定航向大地坐标系角度
			 */
			case (MODE_CASE(INIT)):
			{
				/* 执行天线初始化 */
				ret = mode_initizlize_run(&g_sat_att_ctrl);
				if(ret < 0){
					syslog(LOG_ERR,"Failed to run antenna INIT:%d\n",ret);
					return ret;
				}

			}break;

			/*
			 * 任务循环RUN
			 * 1 数据更新（姿态、L1/L2/L3绝对位置、目标射束角度）
			 * 2 控制量计算
			 * 3 数据发布
			 */
			case (MODE_CASE(RUN)):
			{
				ret = mode_control_attitude_run(&g_sat_att_ctrl);
				if(ret < 0){
					syslog(LOG_ERR,"Failed to control antenna attitude:%d\n",ret);
					return ret;
				}

			}break;

			/*
			 * 停止STOP
			 * 1 关闭PID计算
			 * 2 清空PID过程变量
			 * 3 清空退出任务
			 */
			case (MODE_CASE(STOP)):
			{
				ret = mode_stop_run(&g_sat_att_ctrl);
				if(ret < 0){
					syslog(LOG_ERR,"Failed to stop antenna attitude control:%d\n",ret);
					return ret;
				}
			}break;

			case (IDLE / 10):
			{
				;
			}break;

			default:
			{

			}break;
		}

		/*
		 * 发布姿态控制状态信息
		 */
//		arm_var_f32(float32_t * pSrc,uint32_t blockSize,float32_t * pResult);

//		memset(&(g_sat_att_ctrl.pub_attitude_control_status->msg),0,sizeof(g_sat_att_ctrl.pub_attitude_control_status->msg));

		g_sat_att_ctrl.pub_attitude_control_status->msg.mode 		= g_sat_att_ctrl.mode;
		g_sat_att_ctrl.pub_attitude_control_status->msg.curr_roll 	= g_sat_att_ctrl.beam->attitude.ned_roll;
		g_sat_att_ctrl.pub_attitude_control_status->msg.curr_pitch 	= g_sat_att_ctrl.beam->attitude.ned_pitch;
		g_sat_att_ctrl.pub_attitude_control_status->msg.curr_yaw 	= g_sat_att_ctrl.beam->attitude.ned_yaw;
		g_sat_att_ctrl.pub_attitude_control_status->msg.stable  	= true;

		orb_publish(ORB_ID(attitude_control_status_uorb), g_sat_att_ctrl.pub_attitude_control_status->pub, &(g_sat_att_ctrl.pub_attitude_control_status->msg));

//		 hrt_abstime abs2 = hrt_absolute_time();
//		 t = abs2 - abscurr;
////		 printf("               delta:%d\n",t);
//
//		/* 保存上一次调用的绝对时间 */
//		last = abscurr;
	 }

	return 0;
}

/****************************************************************************
 * Name: sat_att_ctrl_task_create
 *
 * Description:
 *   satellite attitude control task create.
 *
 * Input Parameters:
 *   argc  - number of parameters.
 *   argv  - pointer of parameters.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int sat_att_ctrl_task_create(int argc, char *argv[])
{
	g_sat_att_ctrl.task_pid = task_create(	CONFIG_SATELLITE_ATTITUDE_CONTROL_PROGNAME, \
											CONFIG_SATELLITE_ATTITUDE_CONTROL_PRIORITY, \
											CONFIG_SATELLITE_ATTITUDE_CONTROL_STACKSIZE,\
										sat_att_ctrl_run,
										NULL);
    if (g_sat_att_ctrl.task_pid < 0){
        int errcode = errno;
        syslog(LOG_ERR, "[SAC] ERROR: Failed to start satellite attitude control task: %d\n",errcode);
        return -errcode;
      }

    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void usage(void)
{
	printf("satellite attitude control application\n");
	printf(" <cmd> <start> start the program.\n");
	printf(" <cmd> <stop> stop the program\n");
	printf(" <cmd> <status> print the program status.\n");
	printf(" <cmd> <-w> <-a> write all parameters to storage.\n");
	printf(" <cmd> <-r> <-a> read all the parameters\n");
	printf(" <cmd> <-s> <param> <vlue> set the parameter\n");
	printf("\n");
	printf(" <param> example:\n");
	printf("  rpp:kp of received antenna pitch direction.\n");
	printf("  |||------ p:kp   i:ki   d:kd   f:kf.\n");
	printf("  ||       im:maximal integrator.\n");
	printf("  ||       om:maximal output value.\n");
	printf("  ||\n");
	printf("  ||-------p:pitch direction.\n");
	printf("  | 	   y:yaw direction.\n");
	printf("  |\n");
	printf("  |--------r:received antenna.\n");
	printf("           t:transmit antenna.\n");
	printf("\n");
}


/****************************************************************************
 * satellite_attitude_control_main
 ****************************************************************************/

int
satellite_attitude_control_main(int argc, char *argv[])
{

	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start the application.
	 */
	if (!strcmp(argv[1], "start")) {
		if(g_sat_att_ctrl.should_exit){

			g_sat_att_ctrl.mode 	   = INIT;
			g_sat_att_ctrl.should_exit = false;
			sat_att_ctrl_task_create(argc,argv);
		}else{
			printf("satellite attitude control task already running\n");
		}
		return OK;
	}

	/*
	 * Stop the application.
	 */
	else if (!strcmp(argv[1], "stop")) {

		if(argc == 4){
			//force to quit application
			if(!strcmp(argv[2], "-f")){
				if(!g_sat_att_ctrl.should_exit){

					g_sat_att_ctrl.mode		   = STOP;
					g_sat_att_ctrl.should_exit = true;
				}else{
					printf("satellite attitude control task already exit.\n");
				}

				return OK;
			}
		}
		else{
			//quit application normally
			if(!g_sat_att_ctrl.should_exit){
				g_sat_att_ctrl.mode	= STOP;
			}else{
				printf("satellite attitude control task already exit\n");
			}
			return OK;
		}
	}

	/*
	 * print the application status.
	 */
	else if (!strcmp(argv[1], "status")) {
		printf("satellite attitude control application\n");
		printf("Task Mode:%d\n",g_sat_att_ctrl.mode);
		printf("  [ 1:INIT_SEARCH_HOME   2:INIT_WAIT_COMPLETE   3:INIT_WAIT_COMMAND]\n");
		printf("  [11:RUN_ATTITUDE_CONTROL]\n");
		printf("  [20:STOP]\n");
		printf("  [30:IDLE]\n");
		printf("\n");
		return OK;
	}

	/*
	 * Set parameters of the application.
	 */
	else if (!strcmp(argv[1], "-s")) {
		if(argc != 4){
			usage();
			return -EINVAL;
		}

		/*
		 * received antenna pitch direction
		 */
		if(!strcmp(argv[2], "rpp")){
			g_rxatt_pitch_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "rpi")){
			g_rxatt_pitch_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "rpd")){
			g_rxatt_pitch_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "rpf")){
			g_rxatt_pitch_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "rpim")){
			g_rxatt_pitch_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "rpom")){
			g_rxatt_pitch_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "rpfq")){
			g_rxatt_pitch_pid.params.fCut = atof(argv[3]);
		}

		/*
		 * received antenna yaw direction
		 */
		 else if(!strcmp(argv[2], "ryp")){
			g_rxatt_yaw_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryi")){
			g_rxatt_yaw_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryd")){
			g_rxatt_yaw_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryf")){
			g_rxatt_yaw_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryim")){
			g_rxatt_yaw_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryom")){
			g_rxatt_yaw_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryfq")){
			g_rxatt_yaw_pid.params.fCut = atof(argv[3]);
		}

		/*
		 * received antenna yaw2 direction
		 */
		 else if(!strcmp(argv[2], "ryp2")){
			g_rxatt_yaw2_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryi2")){
			g_rxatt_yaw2_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryd2")){
			g_rxatt_yaw2_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryf2")){
			g_rxatt_yaw2_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryim2")){
			g_rxatt_yaw2_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryom2")){
			g_rxatt_yaw2_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "ryfq2")){
			g_rxatt_yaw2_pid.params.fCut = atof(argv[3]);
		}

		/*
		 * received antenna polar direction
		 */
		 else if(!strcmp(argv[2], "rrp")){
			g_rxatt_polar_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "rri")){
			g_rxatt_polar_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrd")){
			g_rxatt_polar_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrf")){
			g_rxatt_polar_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrim")){
			g_rxatt_polar_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrom")){
			g_rxatt_polar_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrfq")){
			g_rxatt_polar_pid.params.fCut = atof(argv[3]);
		}

		/*
		 * received antenna yaw2 direction
		 */
		 else if(!strcmp(argv[2], "rrp2")){
			g_rxatt_polar2_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "rri2")){
			g_rxatt_polar2_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrd2")){
			g_rxatt_polar2_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrf2")){
			g_rxatt_polar2_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrim2")){
			g_rxatt_polar2_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrom2")){
			g_rxatt_polar2_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "rrfq2")){
			g_rxatt_polar2_pid.params.fCut = atof(argv[3]);
		}

		/*
		 * received antenna T
		 */
		 else if(!strcmp(argv[2], "rtp")){
			g_rxatt_T_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "rti")){
			g_rxatt_T_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "rtd")){
			g_rxatt_T_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "rtf")){
			g_rxatt_T_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "rtim")){
			g_rxatt_T_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "rtom")){
			g_rxatt_T_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "rtfq")){
			g_rxatt_T_pid.params.fCut = atof(argv[3]);
		}

		/*
		 * received antenna fuzzy pitch
		 */
		 else if(!strcmp(argv[2], "frpp")){
			 g_rx_pitch_fuzzy_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "frpi")){
			g_rx_pitch_fuzzy_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "frpd")){
			g_rx_pitch_fuzzy_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "frpf")){
			g_rx_pitch_fuzzy_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "frpim")){
			g_rx_pitch_fuzzy_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "frpom")){
			g_rx_pitch_fuzzy_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "frpfq")){
			g_rx_pitch_fuzzy_pid.params.fCut = atof(argv[3]);
		}else if(!strcmp(argv[2], "frpit")){
			g_rx_pitch_fuzzy_pid.params.i_threshold = atof(argv[3]);
		}

		/*
		 * received antenna fuzzy yaw
		 */
		 else if(!strcmp(argv[2], "fryp")){
			g_rx_yaw_fuzzy_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "fryi")){
			g_rx_yaw_fuzzy_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "fryd")){
			g_rx_yaw_fuzzy_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "fryf")){
			g_rx_yaw_fuzzy_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "fryim")){
			g_rx_yaw_fuzzy_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "fryom")){
			g_rx_yaw_fuzzy_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "fryfq")){
			g_rx_yaw_fuzzy_pid.params.fCut = atof(argv[3]);
		}else if(!strcmp(argv[2], "fryit")){
			g_rx_yaw_fuzzy_pid.params.i_threshold = atof(argv[3]);
		}

		/*
		 * received antenna fuzzy yaw
		 */
		 else if(!strcmp(argv[2], "frrp")){
			g_rx_polar_fuzzy_pid.params.kp = atof(argv[3]);
		}else if(!strcmp(argv[2], "frri")){
			g_rx_polar_fuzzy_pid.params.ki = atof(argv[3]);
		}else if(!strcmp(argv[2], "frrd")){
			g_rx_polar_fuzzy_pid.params.kd = atof(argv[3]);
		}else if(!strcmp(argv[2], "frrf")){
			g_rx_polar_fuzzy_pid.params.kf = atof(argv[3]);
		}else if(!strcmp(argv[2], "frrim")){
			g_rx_polar_fuzzy_pid.params.i_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "frrom")){
			g_rx_polar_fuzzy_pid.params.o_max = atof(argv[3]);
		}else if(!strcmp(argv[2], "frrfq")){
			g_rx_polar_fuzzy_pid.params.fCut = atof(argv[3]);
		}else if(!strcmp(argv[2], "frrit")){
			g_rx_polar_fuzzy_pid.params.i_threshold = atof(argv[3]);
		}


		/*
		 * received antenna target command
		 */
		 else if(!strcmp(argv[2], "ry")){

			 float tmp_yaw = atof(argv[3]);

			 /* 输入参数有效性检查 */
			 if(fabsf(tmp_yaw) < 180.0){
				 g_attitude_cmd.ned_yaw = tmp_yaw;
			 }else{
				 printf("Invalid Parameters:%f!!! Angle Range(-180,180)\n",tmp_yaw);
			 }

		}else if(!strcmp(argv[2], "rp")){

			float tmp_pitch = atof(argv[3]);

			/* 输入参数有效性检查 */
			 if(fabsf(tmp_pitch) < 90.0){
				 g_attitude_cmd.ned_pitch = tmp_pitch;
			 }else{
				 printf("Invalid Parameters:%f!!! Angle Range(-90,90)\n",tmp_pitch);
			 }

		}else if(!strcmp(argv[2], "rr")){
			float tmp_roll = atof(argv[3]);

			/* 输入参数有效性检查 */
			 if(fabsf(tmp_roll) < 90.0){
				 g_attitude_cmd.ned_roll = tmp_roll;
			 }else{
				 printf("Invalid Parameters:%f!!! Angle Range(-90,90)\n",tmp_roll);
			 }
		}else if(!strcmp(argv[2], "rt")){
			float tmp_t = atof(argv[3]);

			/* 输入参数有效性检查 */
			 if(fabsf(tmp_t) < 90.0){
				 g_attitude_cmd.T = tmp_t;
			 }else{
				 printf("Invalid Parameters:%f!!! Angle Range(-90,90)\n",tmp_t);
			 }
		}

		else if(!strcmp(argv[2], "axis")){
			int tmp_t = atoi(argv[3]);

			/* 输入参数有效性检查 */
			 if(abs(tmp_t) < 4){
				 g_orb_pid_turning.msg.axis = tmp_t;
			 }else{
				 printf("Invalid Parameters:%f!!! Angle Range(1,3)\n",tmp_t);
			 }
		}

		/*
		 * received antenna PID switch
		 */
		else if(!strcmp(argv[2], "rypid")){

			int pid_enable = atoi(argv[3]);

			/* 输入参数有效性检查 */
			 if(pid_enable == 1){
				 g_rxatt_yaw_pid.enable = true;
				 g_rxatt_yaw2_pid.enable = true;
				 g_rx_yaw_fuzzy_pid.enable = true;
			 }else if(pid_enable == 0){
				 g_rxatt_yaw_pid.enable = false;
				 g_rxatt_yaw2_pid.enable = false;
				 g_rx_yaw_fuzzy_pid.enable = false;
			 }else{
				 printf("Invalid Parameters!!! Enable:1 Disable:0\n");
			 }

		}else if(!strcmp(argv[2], "rppid")){
			int pid_enable = atoi(argv[3]);

			/* 输入参数有效性检查 */
			 if(pid_enable == 1){
				 g_rxatt_pitch_pid.enable = true;
				 g_rx_pitch_fuzzy_pid.enable = true;
			 }else if(pid_enable == 0){
				 g_rxatt_pitch_pid.enable = false;
				 g_rx_pitch_fuzzy_pid.enable = false;
			 }else{
				 printf("Invalid Parameters!!! Enable:1 Disable:0\n");
			 }
		}else if(!strcmp(argv[2], "rrpid")){

			int pid_enable = atoi(argv[3]);

			/* 输入参数有效性检查 */
			 if(pid_enable == 1){
				 g_rxatt_polar_pid.enable = true;
				 g_rxatt_polar2_pid.enable = true;
				 g_rx_polar_fuzzy_pid.enable = true;
			 }else if(pid_enable == 0){
				 g_rxatt_polar_pid.enable = false;
				 g_rx_polar_fuzzy_pid.enable = false;
			 }else{
				 printf("Invalid Parameters!!! Enable:1 Disable:0\n");
			 }

		}else if(!strcmp(argv[2], "rtpid")){

			int pid_enable = atoi(argv[3]);

			/* 输入参数有效性检查 */
			 if(pid_enable == 1){
				 g_rxatt_T_pid.enable = true;
			 }else if(pid_enable == 0){
				 g_rxatt_T_pid.enable = false;
			 }else{
				 printf("Invalid Parameters!!! Enable:1 Disable:0\n");
			 }

		}

		else{
			usage();
			return -EINVAL;
		}

		return OK;
	}

	/*
	 * Get parameters of the application.
	 */
	else if (!strcmp(argv[1], "-r")) {
		if(argc != 2){
			usage();
			return -EINVAL;
		}

		/*
		 * Print all control parameters
		 */
		printf("\n");
		printf("received antenna\n");
		printf("   |--pitch\n");
		printf("   |     |--kp  :%.3f\n",g_rxatt_pitch_pid.params.kp);
		printf("   |     |--ki  :%.3f\n",g_rxatt_pitch_pid.params.ki);
		printf("   |     |--kd  :%.3f\n",g_rxatt_pitch_pid.params.kd);
		printf("   |     |--kf  :%.3f\n",g_rxatt_pitch_pid.params.kf);
		printf("   |     |--fcut:%.3f\n",g_rxatt_pitch_pid.params.fCut);
		printf("   |     |--max integrator  :%.3f\n",g_rxatt_pitch_pid.params.i_max);
		printf("   |     |--max output value:%.3f\n",g_rxatt_pitch_pid.params.o_max);
		printf("   |--yaw\n");
		printf("   |     |--kp  :%.3f\n",g_rxatt_yaw_pid.params.kp);
		printf("   |     |--ki  :%.3f\n",g_rxatt_yaw_pid.params.ki);
		printf("   |     |--kd  :%.3f\n",g_rxatt_yaw_pid.params.kd);
		printf("   |     |--kf  :%.3f\n",g_rxatt_yaw_pid.params.kf);
		printf("   |     |--fcut:%.3f\n",g_rxatt_yaw_pid.params.fCut);
		printf("   |     |--max integrator  :%.3f\n",g_rxatt_yaw_pid.params.i_max);
		printf("   |     |--max output value:%.3f\n",g_rxatt_yaw_pid.params.o_max);
		printf("   |--yaw2\n");
		printf("   |     |--kp  :%.3f\n",g_rxatt_yaw2_pid.params.kp);
		printf("   |     |--ki  :%.3f\n",g_rxatt_yaw2_pid.params.ki);
		printf("   |     |--kd  :%.3f\n",g_rxatt_yaw2_pid.params.kd);
		printf("   |     |--kf  :%.3f\n",g_rxatt_yaw2_pid.params.kf);
		printf("   |     |--fcut:%.3f\n",g_rxatt_yaw2_pid.params.fCut);
		printf("   |     |--max integrator  :%.3f\n",g_rxatt_yaw2_pid.params.i_max);
		printf("   |     |--max output value:%.3f\n",g_rxatt_yaw2_pid.params.o_max);
		printf("   |--polar\n");
		printf("   |     |--kp  :%.3f\n",g_rxatt_polar_pid.params.kp);
		printf("   |     |--ki  :%.3f\n",g_rxatt_polar_pid.params.ki);
		printf("   |     |--kd  :%.3f\n",g_rxatt_polar_pid.params.kd);
		printf("   |     |--kf  :%.3f\n",g_rxatt_polar_pid.params.kf);
		printf("   |     |--fcut:%.3f\n",g_rxatt_polar_pid.params.fCut);
		printf("   |     |--max integrator  :%.3f\n",g_rxatt_yaw_pid.params.i_max);
		printf("   |     |--max output value:%.3f\n",g_rxatt_yaw_pid.params.o_max);
		printf("   |--polar2\n");
		printf("   |     |--kp  :%.3f\n",g_rxatt_polar2_pid.params.kp);
		printf("   |     |--ki  :%.3f\n",g_rxatt_polar2_pid.params.ki);
		printf("   |     |--kd  :%.3f\n",g_rxatt_polar2_pid.params.kd);
		printf("   |     |--kf  :%.3f\n",g_rxatt_polar2_pid.params.kf);
		printf("   |     |--fcut:%.3f\n",g_rxatt_polar2_pid.params.fCut);
		printf("   |     |--max integrator  :%.3f\n",g_rxatt_polar2_pid.params.i_max);
		printf("   |     |--max output value:%.3f\n",g_rxatt_polar2_pid.params.o_max);
		printf("   |--T\n");
		printf("   |     |--kp  :%.3f\n",g_rxatt_T_pid.params.kp);
		printf("   |     |--ki  :%.3f\n",g_rxatt_T_pid.params.ki);
		printf("   |     |--kd  :%.3f\n",g_rxatt_T_pid.params.kd);
		printf("   |     |--kf  :%.3f\n",g_rxatt_T_pid.params.kf);
		printf("   |     |--fcut:%.3f\n",g_rxatt_T_pid.params.fCut);
		printf("   |     |--max integrator  :%.3f\n",g_rxatt_T_pid.params.i_max);
		printf("   |     |--max output value:%.3f\n",g_rxatt_T_pid.params.o_max);
		printf("   |\n");
		printf("   |--fuzzy[pitch]       [yaw]          [polar]\n");
		printf("   |     |               |              | \n");
		printf("   |     |--kp  :%3.3f   |--kp  :%3.3f  |--kp  :%3.3f\n",g_rx_pitch_fuzzy_pid.params.kp,g_rx_yaw_fuzzy_pid.params.kp,g_rx_polar_fuzzy_pid.params.kp);
		printf("   |     |--ki  :%3.3f   |--ki  :%3.3f  |--ki  :%3.3f\n",g_rx_pitch_fuzzy_pid.params.ki,g_rx_yaw_fuzzy_pid.params.ki,g_rx_polar_fuzzy_pid.params.ki);
		printf("   |     |--kd  :%3.3f   |--kd  :%3.3f  |--kd  :%3.3f\n",g_rx_pitch_fuzzy_pid.params.kd,g_rx_yaw_fuzzy_pid.params.kd,g_rx_polar_fuzzy_pid.params.kd);
		printf("   |     |--kf  :%3.3f   |--kf  :%3.3f  |--kf  :%3.3f\n",g_rx_pitch_fuzzy_pid.params.kf,g_rx_yaw_fuzzy_pid.params.kf,g_rx_polar_fuzzy_pid.params.kf);
		printf("   |     |--fcut:%3.3f  |--fcut:%3.3f |--fcut:%3.3f\n",g_rx_pitch_fuzzy_pid.params.fCut,g_rx_yaw_fuzzy_pid.params.fCut,g_rx_polar_fuzzy_pid.params.fCut);
		printf("   |     |--imax:%3.3f  |--imax:%3.3f |--imax:%3.3f\n",g_rx_pitch_fuzzy_pid.params.i_max,g_rx_yaw_fuzzy_pid.params.i_max,g_rx_polar_fuzzy_pid.params.i_max);
		printf("   |     |--omax:%3.3f  |--omax:%3.3f |--omax:%3.3f\n",g_rx_pitch_fuzzy_pid.params.o_max,g_rx_yaw_fuzzy_pid.params.o_max,g_rx_polar_fuzzy_pid.params.o_max);
		printf("   |     |--ithr:%3.3f   |--ithr:%3.3f  |--ithr:%3.3f\n",g_rx_pitch_fuzzy_pid.params.i_threshold,g_rx_yaw_fuzzy_pid.params.i_threshold,g_rx_polar_fuzzy_pid.params.i_threshold);

		return OK;
	}

	/*
	 * Set parameters of the application.
	 */
	else if (!strcmp(argv[1], "-w")) {
		int nbytes = 0;
		if(argc > 4){
			usage();
			return -EINVAL;
		}

		/*
		 * 保存模糊控制器参数
		 */
		if(!strcmp(argv[2], "fuzzy")){

			// save received antenna fuzzy yaw parameters.
			nbytes = params_save(&(g_rx_yaw_fuzzy_pid.params), ADDR_ATT_RX_CTRL_FUZZY_YAW, sizeof(struct fuzzy_pid_params_t));
			if(nbytes != sizeof(struct fuzzy_pid_params_t)){
				printf("failed to save received antenna fuzzy yaw direction parameters.[%d/%d]\n",nbytes,sizeof(struct fuzzy_pid_params_t));
			}

			// save received antenna fuzzy pitch parameters.
			nbytes = params_save(&(g_rx_pitch_fuzzy_pid.params), ADDR_ATT_RX_CTRL_FUZZY_PITCH, sizeof(struct fuzzy_pid_params_t));
			if(nbytes != sizeof(struct fuzzy_pid_params_t)){
				printf("failed to save received antenna fuzzy pitch direction parameters.[%d/%d]\n",nbytes,sizeof(struct fuzzy_pid_params_t));
			}

			// save received antenna fuzzy polar parameters.
			nbytes = params_save(&(g_rx_polar_fuzzy_pid.params), ADDR_ATT_RX_CTRL_FUZZY_POLAR, sizeof(struct fuzzy_pid_params_t));
			if(nbytes != sizeof(struct fuzzy_pid_params_t)){
				printf("failed to save received antenna fuzzy polar direction parameters.[%d/%d]\n",nbytes,sizeof(struct fuzzy_pid_params_t));
			}

//			printf("fuzzy controller parameters saved!\n");
		}

		else{
			/*
			 * save received antenna pitch direction parameters.
			 */
			nbytes = params_save(&(g_rxatt_pitch_pid.params), ADDR_ATT_RX_CTRL_PITCH, sizeof(struct pid_params_t));
			if(nbytes != sizeof(struct pid_params_t)){
				printf("failed to save received antenna pitch direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
			}

			// save received antenna yaw direction parameters.
			nbytes = params_save(&(g_rxatt_yaw_pid.params), ADDR_ATT_RX_CTRL_YAW, sizeof(struct pid_params_t));
			if(nbytes != sizeof(struct pid_params_t)){
				printf("failed to save received antenna yaw direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
			}

			// save received antenna yaw2 direction parameters.
			nbytes = params_save(&(g_rxatt_yaw2_pid.params), ADDR_ATT_RX_CTRL_YAW2, sizeof(struct pid_params_t));
			if(nbytes != sizeof(struct pid_params_t)){
				printf("failed to save received antenna yaw2 direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
			}

			// save received antenna polar direction parameters.
			nbytes = params_save(&(g_rxatt_polar_pid.params), ADDR_ATT_RX_CTRL_POLAR, sizeof(struct pid_params_t));
			if(nbytes != sizeof(struct pid_params_t)){
				printf("failed to save received antenna polar direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
			}

			// save received antenna polar2 direction parameters.
			nbytes = params_save(&(g_rxatt_polar2_pid.params), ADDR_ATT_RX_CTRL_POLAR2, sizeof(struct pid_params_t));
			if(nbytes != sizeof(struct pid_params_t)){
				printf("failed to save received antenna polar2 direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
			}

			// save received antenna T parameters.
			nbytes = params_save(&(g_rxatt_T_pid.params), ADDR_ATT_RX_CTRL_T, sizeof(struct pid_params_t));
			if(nbytes != sizeof(struct pid_params_t)){
				printf("failed to save received antenna polar2 direction parameters.[%d/%d]\n",nbytes,sizeof(struct pid_params_t));
			}
		}
	}

	/*
	 * Print help information.
	 */
	else{
		usage();
		return -EINVAL;
	}

	return OK;

}
