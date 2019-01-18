/****************************************************************************
 *  apps/my_project/satellite_signal_control/satellite_signal_control_main.c
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

#include <debug.h>
#include <errno.h>

#include <satellite_attitude_control/params_storage.h>
#include <satellite_signal_control/spsa_plus_algorithm.h>
#include <satellite_signal_control/ssc_msg_handle.h>
#include <satellite_signal_control/ssc_mode_run.h>
#include <satellite_signal_control/ssc_mode_stop.h>
#include <satellite_signal_control/ssc_mode_initialize.h>
#include <satellite_signal_control/satellite_signal_control.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*
 * spsa_params_t SPSA算法结构体
 * 公式1(微分方程):
 *	theta_k_next = theta_k +  ak * theta_k_g_k_hat;
 *
 * 公式2(增益序列ak):
 * ak = a/(A + k + 1)^alpha
 *
 * 公式3(增益序列ck)
 * ck = c/(k +1)^gamma
 *
 * 公式4(梯度估计)
 * theta_k_g_k_hat = (tmp_plus - tmp_minus)/(ck * delta_k);
 */
//struct spsa_params_t g_spsa_params = {
//	.a 			= 1.2,
//	.big_a      = 8.6,
//	.c			= 4.8,
//	.alpha		= 0.602,
//	.gamma		= 0.101,
//	.k_az		= -0.3,
//	.k_el		= -0.3,
//	.vc			= -0.35,
//	.base		= 95,
//	.mode		= WAITE_NEXT_POSITION_STABLE,
//	.Smax		= -60,
//	.initialize = false,
//	.theta_plus[AXIS_YAW] = 0,
//	.theta_plus[AXIS_PITCH] = 0,
//	.k						=0,
//	.Soutput				=0,
//	.last_Soutput			=0,
//};

struct spsa_algorithm_t g_spsa_plus_yaw = {

	.run_step = UPDATE_DISTRITUTION,			/* 算法运行步骤 */
	.axis	= SPSA_PLUS_AXIS_YAW,
	/* 衰减参数 */
	.params={
		.ak 	= 0.0,							/* 估计步进系数 */
		.a		= -0.06666667,							/* 估计步进系数,幅度控制参数 */
		.big_a 	= 8.6,							/* 估计步进系数,衰减起点控制参数 */
		.alpha 	= 0.602,						/* 估计步进系数,衰减周期控制参数 */
		.ck 	= 0.0,							/* 随机扰动步进系数 */
		.c		= 0.5,							/* 随机扰动步进系数,幅度控制参数 */
		.gamma 	= 0.8,							/* 随机扰动步进系数,衰减周期控制参数 */
		.b		= -4.66666667,
		.p1     = 0.002048,
		.p2     = 0.2819,
		.p3     = 9.724,
		.k_az	= 0.625,
		.k_el	= 0.625,
		.S_max  = -80,
		.single_range = 0,
		.single_range_max = -80,
		.single_range_min = -100,
	},

	/* 信号变量 */
	.signal_power = {
		.head = 0,								/* 当前(t1时刻)信号强度 */
		.tile = 0,								/* 上刻(t0时刻)信号强度 */
	},

	/* 扰动变量 */
	.disturbance = {
		.delta_k = 0,							/* 随机扰动步进值 */
		.theta_k = 0,							/* 估计步进值 */
		.k		 = 0,							/* 迭代记数 */
	},
};

struct spsa_algorithm_t g_spsa_plus_pitch = {

	.run_step = UPDATE_DISTRITUTION,			/* 算法运行步骤 */
	.axis	= SPSA_PLUS_AXIS_PITCH,
	/* 衰减参数 */
	.params={
		.ak 	= 0.0,							/* 估计步进系数 */
		.a		= -0.06666667,							/* 估计步进系数,幅度控制参数 */
		.big_a 	= 8.6,							/* 估计步进系数,衰减起点控制参数 */
		.alpha 	= 0.602,						/* 估计步进系数,衰减周期控制参数 */
		.ck 	= 0.0,							/* 随机扰动步进系数 */
		.c		= 0.5,							/* 随机扰动步进系数,幅度控制参数 */
		.gamma 	= 0.8,						/* 随机扰动步进系数,衰减周期控制参数 */

		.b		= -4.66666667,
		.p1     = 0.002048,
		.p2     = 0.2819,
		.p3     = 9.724,
		.k_az	= 0.625,
		.k_el	= 0.625,
		.S_max  = -80,
		.single_range = 0,
		.single_range_max = -80,
		.single_range_min = -100,
	},

	/* 信号变量 */
	.signal_power = {
		.head = 0,								/* 当前(t1时刻)信号强度 */
		.tile = 0,								/* 上刻(t0时刻)信号强度 */
	},

	/* 扰动变量 */
	.disturbance = {
		.delta_k = 0,							/* 随机扰动步进值 */
		.theta_k = 0,							/* 估计步进值 */
		.k		 = 0,							/* 迭代记数 */
	},
};


/* 目标卫星参数 */
struct target_satellite_t g_target_satellite ={
	.postion.roll 		= 45,
	.postion.pitch 		= 55,
	.postion.yaw 		= -150,
	.postion.last_roll 	= 45,
	.postion.last_pitch = 55,
	.postion.last_yaw 	= -150,
};

/* advertise data about attitude set points */
struct orb_attitude_set_point_t  g_pub_attitude_sp;

/* advertise data about satellite points */
struct orb_satellite_info_t g_satellite_info= {
		.pass_check = false,
		.enable_check = false,
};

/* subscribe data about beacon infos */
struct orb_beacon_info_t  g_sub_beacon_info = {
	.pass_check = false,
	.enable_check = false,
};

/* subscribe data about attitude control status */
struct orb_attitude_control_status_t g_sub_attitude_control_status = {
		.pass_check = false,
		.enable_check = true,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/
struct sat_signal_ctrl_t  g_sat_sigal_ctrl ={
	.task_pid 						= 0,							 /* 任务PID */
	.mode 							= STOP,							 /* 信标控制模式:详见模式枚举 */
	.should_exit 					= true,							 /* 任务退出flog */
	.fcut							= 40,
	.duty							= 1,
	//
	.spsa_pitch							= &g_spsa_plus_pitch,		 /* SPSA算法 */
	.spsa_yaw							= &g_spsa_plus_yaw,			 /* SPSA算法 */
	//
	.target_satellite				= &g_target_satellite,			 /* 目标卫星 */
	//orb
	.sub_beacon_info				= &g_sub_beacon_info,			 /* 卫星信标消息 */
	.sub_attitude_control_status 	= &g_sub_attitude_control_status,/* 姿态控制状态消息 */
	.sub_satellite_info				= &g_satellite_info,			 /* 卫星位置消息 */
	.pub_attitude_sp				= &g_pub_attitude_sp,			 /* 姿态位置设定消息 */

};



/****************************************************************************
 * Name: sat_signal_ctrl_run
 *
 * Description:
 *   satellite signal control runtime.
 *
 * Input Parameters:
 *   argc  - number of parameters.
 *   argv  - pointer of parameters.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int sat_signal_ctrl_run(int argc, FAR char *argv[])
{

	/***************************************************************************
	 * [参数读取]
	 * 从存储器中读取姿态控制参数
	 **************************************************************************/


	/****************************************************************************
	 * [消息订阅]
	 * 订阅相关uORB消息
	 ****************************************************************************/
	int ret = 0;
	ret = ssc_do_orb_msg_subscribe(&g_sat_sigal_ctrl);
	if(ret < 0){
		syslog(LOG_ERR,"[SSC] Failed to subscribe orb message:%d\n",ret);
		return ret;
	}else{
		syslog(LOG_INFO,"[SSC] Subscribed orb message\n");
	}

	/*****************************************************************************
	 * [消息检查]
	 * 消息订阅检查,进入循环前,检查各个消息状态是否正常(是否正常接收\数据是否正常)
	 *****************************************************************************/
	ret = ssc_do_orb_msg_pre_check(&g_sat_sigal_ctrl);
	if(ret < 0){
		syslog(LOG_ERR,"[SSC] Failed to pre-check message:%d\n",ret);
		return ret;
	}else{
		syslog(LOG_INFO,"[SSC] Pass to check orb message\n");
	}

	/******************************************************************************
	 * [消息登记]
	 * 发布姿态控制目标(发布的周期由控制周期决定)
	 ******************************************************************************/
	ret = ssc_do_orb_msg_advertise(&g_sat_sigal_ctrl);
	if(ret < 0){
		syslog(LOG_ERR,"[SSC] Failed to advertised orb message:%d\n",ret);
		return ret;
	}else{
		syslog(LOG_INFO,"[SSC] Advertised orb message\n");
	}

	/******************************************************************************
	 * [数据处理]
	 * 1.数据更新:
	 * 			1) uORB订阅电机位置/IMU姿态数据/目标位置信息;
	 * 2.模式处理:
	 * 			1) INIT[初始化模式]:等待姿态控制稳定,等待进入[解算模式];
	 * 			2) Run[解算模式]:依据卫星信标和卫星空间指向,进行信号闭环;
	 * 			3) Stop[停止模式]:停止信号闭环控制,关闭相关数据;
	 *
	 ******************************************************************************/
	int duty_cycle = 0;
	/* 默认初始等待姿态稳定 */
	g_sat_sigal_ctrl.mode = INIT_WAIT_ATTITUDE_COMPLETED;
	static hrt_abstime last;
	 while(!g_sat_sigal_ctrl.should_exit){

		 hrt_abstime abscurr = hrt_absolute_time();

		/*
		 * 1 数据更新
		 *  1.1 poll UORB数据更新，超时设定100ms
		 *  1.2 timeout异常处理
		 *  1.3 poll失败异常处理
		 */
		 ret = ssc_do_orb_msg_poll(&g_sat_sigal_ctrl);
		 if(ret <= 0){
			 syslog(LOG_ERR,"ERROR:failed to poll message : %d\n", ret);
		 }

//		 if(duty_cycle++ > g_sat_sigal_ctrl.duty){
//			 duty_cycle = 0;

			/*
			 * 2 数据处理
			 *  2.1 模式选择
			 *  2.2 执行对应模式逻辑
			 */
			switch(MODE_CASE(g_sat_sigal_ctrl.mode)){

				/*
				 * 初始化INITIALIZE
				 * 1 等待天线姿态复位完成
				 * 	1.1 等待姿态进入跟踪模式
				 * 	1.2 等待接收
				 */
				case (MODE_CASE(INIT)):
				{
					/* 执行天线初始化 */
					ret = ssc_mode_initizlize_run(&g_sat_sigal_ctrl);
					if(ret < 0){
						syslog(LOG_ERR,"Failed to run antenna initialize:%d\n",ret);
						return ret;
					}

				}break;

				/*
				 * 任务循环RUN
				 * 1 数据更新（信标信息,当前位置,标姿态位置）
				 * 2 执行搜星算法,输出控制量
				 * 3 控制数据发布
				 */
				case (MODE_CASE(RUN)):
				{
					ret = mode_control_signal_run(&g_sat_sigal_ctrl);
					if(ret < 0){
						syslog(LOG_ERR,"Failed to control satellite signal:%d\n",ret);
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
					ret = mode_stop_run(&g_sat_sigal_ctrl);
					if(ret < 0){
						syslog(LOG_ERR,"Failed to stop satellite signal control:%d\n",ret);
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
			 int t = abscurr - last;
//			 printf("               delta:%d\n",t);

			/* 保存上一次调用的绝对时间 */
			last = abscurr;
		 }
//	}

	return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
static void usage(void)
{
	printf("satellite signal_control application\n");
	printf("[<satellite_signal_control> <start>] start the program.\n");
	printf("{<satellite_signal_control> <stop>] stop the program\n");
}

/****************************************************************************
 * Name: sat_signal_ctrl_task_create
 *
 * Description:
 *   satellite signal control task create.
 *
 * Input Parameters:
 *   argc  - number of parameters.
 *   argv  - pointer of parameters.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int sat_signal_ctrl_task_create(int argc, char *argv[])
{
	g_sat_sigal_ctrl.task_pid = task_create(CONFIG_SATELLITE_SIGNAL_CONTROL_PROGNAME, \
											CONFIG_SATELLITE_SIGNAL_CONTROL_PRIORITY, \
											CONFIG_SATELLITE_SIGNAL_CONTROL_STACKSIZE,\
											sat_signal_ctrl_run,
											NULL);
    if (g_sat_sigal_ctrl.task_pid < 0){

        syslog(LOG_ERR, "[SSC] ERROR: Failed to start satellite signal control task: %d\n",g_sat_sigal_ctrl.task_pid);

        return g_sat_sigal_ctrl.task_pid;

      }

    return OK;
}


/****************************************************************************
 * satellite_signal_control_main
 ****************************************************************************/

int
satellite_signal_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start the application.
	 */
	if (!strcmp(argv[1], "start")) {

		if(g_sat_sigal_ctrl.should_exit){

			g_sat_sigal_ctrl.mode = INIT;

			g_sat_sigal_ctrl.should_exit = false;

			sat_signal_ctrl_task_create(argc,argv);

		}else{

			printf("satellite signal control task already running\n");

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

				if(!g_sat_sigal_ctrl.should_exit){

					g_sat_sigal_ctrl.mode = STOP;

					g_sat_sigal_ctrl.should_exit = true;

				}else{

					printf("satellite signal control task already exit.\n");

				}

				return OK;
			}

		}


		/*
		 * Stop the application.
		 */
		else if (!strcmp(argv[1], "stop")) {

			if(argc == 4){

				//force to quit application
				if(!strcmp(argv[2], "-f")){

					if(!g_sat_sigal_ctrl.should_exit){

						g_sat_sigal_ctrl.mode = STOP;

						g_sat_sigal_ctrl.should_exit = true;

					}else{

						printf("satellite signal control task already exit.\n");

					}

					return OK;
				}

			}
		}
		else{
			//quit application normally
			if(!g_sat_sigal_ctrl.should_exit){

				g_sat_sigal_ctrl.mode = STOP;

			}else{

				printf("satellite signal control task already exit\n");

			}
			return OK;
		}
	}

	/*
	 * get the application status.
	 */
	else if (!strcmp(argv[1], "status")) {
		printf("satellite signal control application\n");
		printf("Task Mode:%d\n",g_sat_sigal_ctrl.mode);
		printf("  [ 1:INIT_MSG_AND_PARAMS   2:INIT_WAIT_ATTITUDE_COMPLETED   3:INIT_WAIT_COMMAND]\n");
		printf("  [11:RUN_POSITION_SEARCH  12:RUN_DISTURBANCE_SEARCH  13:RUN_SIGNAL_SEARCH]\n");
		printf("  [20:STOP]\n");
		printf("  [30:IDLE]\n");
		printf("\n");
		return OK;
	}

	/*
	 * Set the application.
	 */
	else if (!strcmp(argv[1], "-s")) {

		if(argc <= 4){

			//force to quit application
			if(!strcmp(argv[2], "ya")){

				g_sat_sigal_ctrl.spsa_yaw->params.a = atof(argv[3]);

				return OK;
			}
			//force to quit application
			else if(!strcmp(argv[2], "yc")){

				g_sat_sigal_ctrl.spsa_yaw->params.c = atof(argv[3]);

				return OK;
			}
			//force to quit application
			else if(!strcmp(argv[2], "yA")){

				g_sat_sigal_ctrl.spsa_yaw->params.big_a = atof(argv[3]);

				return OK;
			}

			//force to quit application
			else if(!strcmp(argv[2], "pa")){

				g_sat_sigal_ctrl.spsa_pitch->params.a = atof(argv[3]);

				return OK;
			}
			//force to quit application
			else if(!strcmp(argv[2], "pc")){

				g_sat_sigal_ctrl.spsa_pitch->params.c = atof(argv[3]);

				return OK;
			}
			//force to quit application
			else if(!strcmp(argv[2], "pA")){

				g_sat_sigal_ctrl.spsa_pitch->params.big_a = atof(argv[3]);

				return OK;
			}
			//force to quit application
			else if(!strcmp(argv[2], "freq")){

				g_sat_sigal_ctrl.fcut = atof(argv[3]);

				return OK;
			}

			//force to quit application
			else if(!strcmp(argv[2], "duty")){

				g_sat_sigal_ctrl.duty = atoi(argv[3]);

				return OK;
			}

			else if(!strcmp(argv[2], "ry")){

				 float tmp_yaw = atof(argv[3]);

				 /* 输入参数有效性检查 */
				 if(fabsf(tmp_yaw) < 180.0){
					 g_sat_sigal_ctrl.target_satellite->postion.yaw = tmp_yaw;
				 }else{
					 printf("Invalid Parameters:%f!!! Angle Range(-180,180)\n",tmp_yaw);
				 }

			}

			else if(!strcmp(argv[2], "rp")){

				float tmp_pitch = atof(argv[3]);

				/* 输入参数有效性检查 */
				 if(fabsf(tmp_pitch) < 90.0){
					 g_sat_sigal_ctrl.target_satellite->postion.pitch = tmp_pitch;
				 }else{
					 printf("Invalid Parameters:%f!!! Angle Range(-90,90)\n",tmp_pitch);
				 }

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
