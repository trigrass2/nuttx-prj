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
#include <satellite_attitude_control/sac_beam_convert.h>
#include <satellite_attitude_control/sac_ctrl_stabilize.h>
#include <satellite_attitude_control/sac_ctrl_output_mixer.h>
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


///****************************************************************************
// * Name: mode_control_attitude_run
// *
// * Description:
// *   satellite attitude control initialize.
// *
// * Input Parameters:
// *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
// *
// * Returned Value:
// *   the status of program.
// *
// ****************************************************************************/
//float vel = 0;float ppa=0;
//int mode_control_attitude_run(void *psat_att_ctrl)
//{
//	struct sat_att_ctrl_t *sac = psat_att_ctrl;
//	int ret = 0;
//
//	/* 循环计时,用于积分控制时间dt */
//
//	/* 依据<电机位置><IMU>计算当前波束空间位置 */
//	ret = beam_convert_position_to_euler(sac);
//
//	if(ret < 0){
//		syslog(LOG_ERR,"[SAC] ERROR:satellite antenna beam convert error:%d\n",ret);
//	}
//
//	/* 依据当前载体姿态信息,将波束坐标装换到大地坐标系 */
//	ret = beam_convert_body_to_ned(sac);
//
//	if(ret < 0){
//		syslog(LOG_ERR,"[SAC] ERROR:satellite antenna beam convert coordinate error:%d\n",ret);
//	}
//
//	/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
//	control_stabilize_fuzzy_pid(sac);
//
//	/*
//	 * 控制模型输出
//	 * 当T角小于20度采用yaw2控制量
//	 * 当T角大于20度采用yaw 控制量
//	 */
//	float L1_SPEED = 0.0;
//	float L2_SPEED = 0.0;
//	float L3_SPEED = 0.0;
//
//	/*
//	 * L2 SPEED
//	 */
//	if(sac->beam->attitude.T < 20.0){
//		L2_SPEED = sac->rxatt_yaw2->info.desired;
//	}else{
//		L2_SPEED = sac->rxatt_yaw->info.desired;
//	}
//
//	/*
//	 * L1 SPEED
//	 */
//	L1_SPEED = L2_SPEED + sac->rxatt_pitch->info.desired;
//
//	/*
//	 * L3 SPEED
//	 */
//	if(sac->beam->attitude.T < 35.0){
//		L3_SPEED = L2_SPEED;// + sac->rxatt_polar->info.desired;
//	}else{
//		L3_SPEED = L2_SPEED;// + sac->rxatt_polar2->info.desired;
//	}
//
//	/*
//	 * 软件限位
//	 * 当出现T角超过50度且当前电机:
//	 * 	1)如果L1相对于L2的运动速度大于0,此时强行设置为L1为L2的速度;
//	 * 	2)如果L1相对于L2的运动速度小于0,此时为正常情况不处理;
//	 * 当出现T角小于0度:
//	 *  1)如果L1相对于L2的运动速度小于0,此时强行设置为L2为L1的速度;
//	 *  2)如果L1相对于L2的运动速度大于0,此时为正常情况不处理;
//	 */
//
////	if(sac->beam->attitude.T > 50.0){
////		if((L1_SPEED - L2_SPEED) > 0.0){
////			L1_SPEED = 0.0;
////		}
////	}else if(sac->beam->attitude.T < 0.0){
////		if((L1_SPEED - L2_SPEED) < 0.0){
////			L2_SPEED = 0.0;
////		}
////	}
//
//	static int cnt = 0;
//	static int val = 0;
//	cnt++;
//
//	if(cnt%5 == 0)
//	{
//		val++;
//		vel = fabsf(sin(((float)(val%360))/57.3)*100)+ 5;
////		ppa = fabsf(sin(((float)((val/10)%360))/57.3)*50);
////
////		if(ppa > 45) sac->cmd->ned_pitch = 45;
////		else if(ppa > 25) sac->cmd->ned_pitch = 25;
////		else if(ppa > 15) sac->cmd->ned_pitch = 15;
////		else sac->cmd->ned_pitch = 10;
//	}
//
//
//
////	uint32_t tnow = (uint32_t) (hrt_absolute_time()/1000);
////	float vel = fabsf(50*sin((tnow % 360)*180/3.141592654));
//
//	/* 发布L1/L2/L3控制输出
//	 *
//	 * 电机ID就是cmd数组的下标[0,5]
//	 * 0x03:电机找零(设置任意一个电机,所有盘都会找零)
//	 * 0x04:设置绝对位置[0.0,360.0]
//	 * 0x0C:设置相对位置[-1080.0,1080.0]
//	 * 0x14:设置速度[-100.0,100.0]
//	 */
//	memset(&(sac->pub_motors_cmd->msg),0,sizeof(sac->pub_motors_cmd->msg));
//
//	sac->pub_motors_cmd->msg.cmd[0] 			= 0x04;								/* 设置电机1PWM速度 */
//	sac->pub_motors_cmd->msg.params[0].fdata 	= vel;//L1_SPEED;   						/* 具体速度值 */
//
//	sac->pub_motors_cmd->msg.cmd[1] 			= 0x14;								/* 设置电机2PWM速度 */
//	sac->pub_motors_cmd->msg.params[1].fdata 	= L2_SPEED;							/* 具体速度值 */
//
//	sac->pub_motors_cmd->msg.cmd[2] 			= 0x14;								/* 设置电机3PWM速度 */
//	sac->pub_motors_cmd->msg.params[2].fdata 	= L3_SPEED;							/* 具体速度值 */
//
//	orb_publish(ORB_ID(pc_motor_cmd_uorb), sac->pub_motors_cmd->pub, &(sac->pub_motors_cmd->msg));
//
//	return OK;
//}

/****************************************************************************
 * Name: mode_control_attitude_run
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
float vel = 0;
int mode_control_attitude_run(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;
	int ret = 0;
	float L1_SPEED = 0.0,L2_SPEED = 0.0,L3_SPEED = 0.0;

	/* 循环计时,用于积分控制时间dt */

	/* 依据<电机位置><IMU>计算当前波束空间位置 */
	ret = beam_convert_position_to_euler(sac);

	if(ret < 0){
		syslog(LOG_ERR,"[SAC] ERROR:satellite antenna beam convert error:%d\n",ret);
	}

	/* 依据当前载体姿态信息,将波束坐标装换到大地坐标系 */
	ret = beam_convert_beam_to_ned(sac);//beam_convert_body_to_ned(sac);

	if(ret < 0){
		syslog(LOG_ERR,"[SAC] ERROR:satellite antenna beam convert coordinate error:%d\n",ret);
	}

//	/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
//	control_stabilize_linear_pid(sac);
//
//	/*控制模型输出*/
//	mixer_output_linear_pid(sac,&L1_SPEED,&L2_SPEED,&L3_SPEED);

	/* 执行射束姿态闭环控制逻辑 <俯仰使能><航向使能>*/
	control_stabilize_fuzzy_pid(sac);

	/*控制模型输出*/
	mixer_output_fuzzy_pid(sac,&L1_SPEED,&L2_SPEED,&L3_SPEED);

	/*
	 * 软件限位
	 * 当出现T角超过50度且当前电机:
	 * 	1)如果L1相对于L2的运动速度大于0,此时强行设置为L1为L2的速度;
	 * 	2)如果L1相对于L2的运动速度小于0,此时为正常情况不处理;
	 * 当出现T角小于0度:
	 *  1)如果L1相对于L2的运动速度小于0,此时强行设置为L2为L1的速度;
	 *  2)如果L1相对于L2的运动速度大于0,此时为正常情况不处理;
	 */

//	if(sac->beam->attitude.T > 50.0){
//		if((L1_SPEED - L2_SPEED) > 0.0){
//			L1_SPEED = 0.0;
//		}
//	}else if(sac->beam->attitude.T < 0.0){
//		if((L1_SPEED - L2_SPEED) < 0.0){
//			L2_SPEED = 0.0;
//		}
//	}

	/* 发布L1/L2/L3控制输出
	 *
	 * 电机ID就是cmd数组的下标[0,5]
	 * 0x03:电机找零(设置任意一个电机,所有盘都会找零)
	 * 0x04:设置绝对位置[0.0,360.0]
	 * 0x0C:设置相对位置[-1080.0,1080.0]
	 * 0x14:设置速度[-100.0,100.0]
	 */
	memset(&(sac->pub_motors_cmd->msg),0,sizeof(sac->pub_motors_cmd->msg));

	sac->pub_motors_cmd->msg.cmd[0] 			= 0x14;								/* 设置电机1PWM速度 */
	sac->pub_motors_cmd->msg.params[0].fdata 	= L1_SPEED;   						/* 具体速度值 */

	sac->pub_motors_cmd->msg.cmd[1] 			= 0x14;								/* 设置电机2PWM速度 */
	sac->pub_motors_cmd->msg.params[1].fdata 	= L2_SPEED;							/* 具体速度值 */

	sac->pub_motors_cmd->msg.cmd[2] 			= 0x14;								/* 设置电机3PWM速度 */
	sac->pub_motors_cmd->msg.params[2].fdata 	= L3_SPEED;							/* 具体速度值 */

	orb_publish(ORB_ID(pc_motor_cmd_uorb), sac->pub_motors_cmd->pub, &(sac->pub_motors_cmd->msg));

	return OK;
}

///****************************************************************************
// * Name: mode_control_attitude_run
// *
// * Description:
// *   satellite attitude control initialize.
// *
// * Input Parameters:
// *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
// *
// * Returned Value:
// *   the status of program.
// *
// ****************************************************************************/
//int mode_control_attitude_run(void *psat_att_ctrl)
//{
//	struct sat_att_ctrl_t *sac = psat_att_ctrl;
//	int ret = 0;
//
//	/* 循环计时,用于积分控制时间dt */
//
//	/* 依据<电机位置><IMU>计算当前波束空间位置 */
//	ret = beam_convert_position_to_euler(sac);
//
//	if(ret < 0){
//		syslog(LOG_ERR,"[SAC] ERROR:satellite antenna beam convert error:%d\n",ret);
//	}
//
//	/* 依据当前载体姿态信息,将波束坐标装换到大地坐标系 */
//	ret = beam_convert_body_to_ned(sac);
//
//	if(ret < 0){
//		syslog(LOG_ERR,"[SAC] ERROR:satellite antenna beam convert coordinate error:%d\n",ret);
//	}
//
//	/* 执行T闭环控制逻辑 <俯仰使能><航向使能>*/
//
//	control_t_stabilize(sac);
//	control_stabilize(sac);
//	/*
//	 * 控制模型输出
//	 * 当T角小于20度采用yaw2控制量
//	 * 当T角大于20度采用yaw 控制量
//	 */
//	float L1_SPEED = 0.0;
//	float L2_SPEED = 0.0;
//	float L3_SPEED = 0.0;
//
////	/*
////	 * L2 SPEED
////	 */
////	if(sac->beam->attitude.T < 20.0){
////		L2_SPEED = sac->rxatt_yaw2->info.desired;
////	}else{
////		L2_SPEED = sac->rxatt_yaw->info.desired;
////	}
////
////	/*
////	 * L1 SPEED
////	 */
////	L1_SPEED = L2_SPEED + sac->rxatt_pitch->info.desired;
////
////
////	/*
////	 * L3 SPEED
////	 */
////	if(sac->beam->attitude.T < 35.0){
////		L3_SPEED = L2_SPEED;// + sac->rxatt_polar->info.desired;
////	}else{
////		L3_SPEED = L2_SPEED;// + sac->rxatt_polar2->info.desired;
////	}
//
//	/*
//	 * L1 SPEED
//	 */
//	L1_SPEED = sac->rxatt_T->info.desired;
//
//	/*
//	 * L2 SPEED
//	 */
//	if(sac->beam->attitude.T < 20.0){
//		L2_SPEED = sac->rxatt_yaw2->info.desired;
//	}else{
//		L2_SPEED = sac->rxatt_yaw->info.desired;
//	}
//
//	/* 发布L1/L2/L3控制输出
//	 *
//	 * 电机ID就是cmd数组的下标[0,5]
//	 * 0x03:电机找零(设置任意一个电机,所有盘都会找零)
//	 * 0x04:设置绝对位置[0.0,360.0]
//	 * 0x0C:设置相对位置[-1080.0,1080.0]
//	 * 0x14:设置速度[-100.0,100.0]
//	 */
//	memset(&(sac->pub_motors_cmd->msg),0,sizeof(sac->pub_motors_cmd->msg));
//
//	sac->pub_motors_cmd->msg.cmd[0] 			= 0x14;								/* 设置电机1PWM速度 */
//	sac->pub_motors_cmd->msg.params[0].fdata 	= L2_SPEED;//L1_SPEED;   						/* 具体速度值 */
//
//	sac->pub_motors_cmd->msg.cmd[1] 			= 0x14;								/* 设置电机2PWM速度 */
//	sac->pub_motors_cmd->msg.params[1].fdata 	= L2_SPEED;							/* 具体速度值 */
//
//	sac->pub_motors_cmd->msg.cmd[2] 			= 0x14;								/* 设置电机3PWM速度 */
//	sac->pub_motors_cmd->msg.params[2].fdata 	= L3_SPEED;							/* 具体速度值 */
//
//	orb_publish(ORB_ID(pc_motor_cmd_uorb), sac->pub_motors_cmd->pub, &(sac->pub_motors_cmd->msg));
//
//	return OK;
//}
