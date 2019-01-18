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
#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <math.h>

#include <nuttx/config.h>
#include <nuttx/init.h>
#include <nuttx/timers/drv_hrt.h>

#include <satellite_attitude_control/message_handle.h>
#include <satellite_attitude_control/sac_beam_convert.h>
#include <satellite_attitude_control/satellite_attitude_control.h>

#include <imu/imu_rotate.h>
#include <imu/imu_att_rotate.h>

#define MIN_ANGLE_T 	 (0.0)
#define MAX_ANGLE_T 	 (51.0)

#define MAX_ANGLE_PITCH  (90.0)
#define MIN_ANGLE_PITCH  (-MAX_ANGLE_PITCH)

#define MAX_ANGLE_YAW  	 (180.0)
#define MIN_ANGLE_YAW  	 (-MAX_ANGLE_YAW)


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: beam_convert_position_to_euler
 *
 * Description:
 *   convert motors position to euler angle.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int beam_convert_position_to_euler(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;
	float temp_l1_position 	= 0.0;
	float temp_l2_position 	= 0.0;
	float temp_l3_position	= 0.0;
	float temp_angle_T	  	= 0.0;
	float temp_angle_L2_L3	= 0.0;

	/* 设置L2盘的位置为基准位置 */
	sac->beam->attitude.BASE = sac->sub_motors_fb->position[1];

	/*
	 * 设置T角的值为L1相对L2的夹角
	 * 因为T不能为负,所以在正常控制中L1的角度始终大于L2的角度!!!
	 * 只有在初始化找零的时候可能出现L2大于L1(应为L2可能出现在180.0附近)
	 */

	temp_l1_position = sac->sub_motors_fb->position[0];//-180 ~ +180
	temp_l2_position = sac->sub_motors_fb->position[1];//-180 ~ +180
	temp_l3_position = sac->sub_motors_fb->position[2];//-180 ~ +180
	/*
	 * 区分同号和异号
	 */

	/*
	 * 如果是异号
	 * 出现L1与L2差值符号反向,差值突变两种问题
	 */
	if((temp_l1_position * temp_l2_position) < 0){

		/* 差值大于180.0度,出现差值突变现象 */
		if((temp_l1_position - temp_l2_position) >= 180.0){
			temp_angle_T =  (temp_l1_position - temp_l2_position) - 360.0;
		}

		/* 差值小于-180.0度,出现差值突变并且符号反向现象 */
		else if((temp_l1_position - temp_l2_position) <= -180.0){

			temp_angle_T = 360.0 + (temp_l1_position - temp_l2_position);
		}
		else{
		/* 差值小于180.0度 */
			temp_angle_T = temp_l1_position - temp_l2_position;
		}

	}
	/* 如果是同号 */
	else{

		/* 差值的绝对值一定小于180.0度 */
		temp_angle_T = temp_l1_position - temp_l2_position;

	}

	/* 确保L2在L1的左边 */
	temp_angle_T *=-1;

	/*
	 * 幅度限制确保T[0.0,50.0]
	 */

	if(temp_angle_T > MAX_ANGLE_T){

		sac->beam->attitude.T = MAX_ANGLE_T;
	}
	else if(temp_angle_T < MIN_ANGLE_T){
		sac->beam->attitude.T = MIN_ANGLE_T;
	}
	else{
		sac->beam->attitude.T = temp_angle_T;
	}



	/*
	 * 如果是异号
	 * 出现L1与L2差值符号反向,差值突变两种问题
	 */
	if((temp_l3_position * temp_l2_position) < 0){

		/* 差值的绝对值大于180.0度,出现差值突变现象 */
		if((temp_l3_position - temp_l2_position) >= 180.0){
			temp_angle_L2_L3 =  (temp_l3_position - temp_l2_position) - 360.0;
		}

		/* 差值的绝对值小于-180.0度,出现差值突变并且符号反向现象 */
		else if((temp_l3_position - temp_l2_position) <= -180.0){

			temp_angle_L2_L3 = 360.0 + (temp_l3_position - temp_l2_position);
		}
		else{
		/* 差值的绝对值小于180.0度 */
			temp_angle_L2_L3 = temp_l3_position - temp_l2_position;
		}

	}
	/* 如果是同号 */
	else{

		/* 差值的绝对值一定小于180.0度 */
		temp_angle_L2_L3 = temp_l3_position - temp_l2_position;

	}


	sac->beam->attitude.L2_L3 = -temp_angle_L2_L3;


	/* 四阶变量累乘 */
	double factorial_first  = sac->beam->attitude.T;
	double factorial_second = sac->beam->attitude.T * factorial_first;
	double factorial_third  = sac->beam->attitude.T * factorial_second;
	double factorial_fourth = sac->beam->attitude.T * factorial_third;
	double factorial_fiveth = sac->beam->attitude.T * factorial_fourth;
	double factorial_sixth  = sac->beam->attitude.T * factorial_fiveth;

	/* 多项式装换为射束的偏航角 */
	sac->beam->attitude.t_roll  =(float)( sac->beam->params.L\
							   +sac->beam->params.A1 * factorial_first\
							   +sac->beam->params.A2 * factorial_second\
							   +sac->beam->params.A3 * factorial_third\
							   +sac->beam->params.A4 * factorial_fourth\
							   +sac->beam->params.A5 * factorial_fiveth\
							   +sac->beam->params.A6 * factorial_sixth);

	/* 多项式装换为射束的俯仰角 */
	sac->beam->attitude.t_pitch =(float)( sac->beam->params.M\
							   +sac->beam->params.B1 * factorial_first\
							   +sac->beam->params.B2 * factorial_second\
							   +sac->beam->params.B3 * factorial_third\
							   +sac->beam->params.B4 * factorial_fourth\
							   +sac->beam->params.B5 * factorial_fiveth\
							   +sac->beam->params.B6 * factorial_sixth);

	/* 多项式装换为射束的偏航角 */
	sac->beam->attitude.t_yaw  =(float)( sac->beam->params.N\
							   +sac->beam->params.C1 * factorial_first\
							   +sac->beam->params.C2 * factorial_second\
							   +sac->beam->params.C3 * factorial_third\
							   +sac->beam->params.C4 * factorial_fourth\
							   +sac->beam->params.C5 * factorial_fiveth\
							   +sac->beam->params.C6 * factorial_sixth);

	return OK;
}

/****************************************************************************
 * Name: beam_convert_beam_to_ned
 *
 * Description:
 *   convert antenna coordinate system ned coordinate system.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int beam_convert_beam_to_ned(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;

	/*
	 * 首先将天线坐标转换到载体坐标中
	 * 相当于在t_yaw的基础上逆时针旋转L2转动的角度
	 */
	sac->beam->attitude.body_yaw 	= sac->beam->attitude.BASE + sac->beam->attitude.t_yaw;
	sac->beam->attitude.body_pitch 	= sac->beam->attitude.t_pitch;
	sac->beam->attitude.body_roll 	= sac->beam->attitude.t_roll;

	/*
	 * 航向在t_yaw和L2角度叠加后需要处理成[-180,180]
	 */
	if(sac->beam->attitude.body_yaw > 180.0 )sac->beam->attitude.body_yaw -= 360.0;

	/*
	 * 将转换到载体坐标系的姿态装换为单位向量,用于坐标旋转
	 */
	float b_x=0,b_y=0,b_z=0;

	b_x = sin(sac->beam->attitude.body_pitch * M_PI / 180.0) * cos(sac->beam->attitude.body_yaw * M_PI / 180.0);
	b_y = sin(sac->beam->attitude.body_pitch * M_PI / 180.0) * sin(sac->beam->attitude.body_yaw * M_PI / 180.0);
	b_z = cos(sac->beam->attitude.body_pitch * M_PI / 180.0);

	/*
	 * 将载体坐标旋转到大地坐标系
	 */
	float n_x=0,n_y=0,n_z=0;

	imu_rotate(b_x, b_y, b_z, &n_x, &n_y, &n_z);


	float a1,b1,c1,a2,b2,c2;
	imu_att_rotate(b_x, b_y, b_z,&a1, &b1, &c1, &a2,&b2, &c2);


	if(n_x <  0.000001 && n_x > 0)n_x =  0.000001;
	else if(n_x > -0.000001 && n_x < 0)n_x = -0.000001;

	float tmp_pitch=0,tmp_yaw=0,b_roll=0;

	tmp_pitch = asin(n_z) * 180 / M_PI;			//
	tmp_yaw = atan2(n_y,n_x) * 180 / M_PI;
	if(tmp_pitch > 90.0) tmp_pitch -=180.0;

	sac->beam->attitude.ned_roll 	= sac->beam->attitude.body_roll;
	sac->beam->attitude.ned_pitch 	= tmp_pitch;
	sac->beam->attitude.ned_yaw 	= tmp_yaw;


	//printf("bx:%6.2f by:%6.2f bz:%6.2f nx:%6.2f ny:%6.2f nz:%6.2f\n", b_x, b_y,b_z, n_x, n_y,n_z);

	/*
	 * 拷贝射束姿态信息到uORB,待发布
	 */
	sac->pub_beam_attitude->msg.roll 	= sac->beam->attitude.ned_roll;			//射束融合姿态后的roll
	sac->pub_beam_attitude->msg.pitch 	= sac->beam->attitude.ned_pitch;		//射束融合姿态后的pitch
	sac->pub_beam_attitude->msg.yaw 	= sac->beam->attitude.ned_yaw;			//射束融合姿态后的yaw

	sac->pub_beam_attitude->msg.val1 	= sac->beam->attitude.T;				//T角
	sac->pub_beam_attitude->msg.val2 	= sac->beam->attitude.t_pitch;			//T角转换后的pitch
	sac->pub_beam_attitude->msg.val3 	= sac->beam->attitude.t_yaw;			//T角转换后的yaw

	/*
	 * 发布射束姿态信息
	 */
	orb_publish(ORB_ID(beam_attitude_uorb), sac->pub_beam_attitude->pub, &(sac->pub_beam_attitude->msg));
	return OK;
}

/****************************************************************************
 * Name: beam_convert_body_to_ned
 *
 * Description:
 *   convert body coordinate system ned coordinate system.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
//int beam_convert_body_to_ned(void *psat_att_ctrl)
//{
//	struct sat_att_ctrl_t *sac = psat_att_ctrl;
//	float tmp = 0.0;
//	float beam_n_roll = 0.0,beam_n_pitch = 0.0,beam_n_yaw = 0.0;
//	/*
//	 * 调用旋转矩阵,将天线坐标系转换到大地坐标系
//	 */
//	imu_rotate(0.0,sac->beam->attitude.t_pitch,sac->beam->attitude.t_yaw, &beam_n_roll, &beam_n_pitch, &beam_n_yaw);
//	printf("0   %6.2f    %6.2f    %6.2f    %6.2f    %6.2f \n", sac->beam->attitude.t_pitch, sac->beam->attitude.t_yaw,beam_n_roll, beam_n_pitch, beam_n_yaw);
//
//	/*
//	 * pitch
//	 * 因为射束的俯仰角(T角转化后描述的是与Z轴的夹角)坐标与姿态坐标方向相('抬头'为正,'低头'为负),所以不能直接进行叠加
//	 * T角转化后的俯仰角描述的是与Z轴的夹角:与Z轴到X-Y平面角度依次[10,80]
//	 * 姿态坐标中俯仰角描述的是与X-Y平面的夹角与Z轴正方向的夹角依次是[0,90],与Z轴负方向的夹角依次是[0,-90]
//	 * 所以叠加后的俯仰角度在大地坐标系中有效范围就是[-80,170],但是只是可能值,实际应该是[姿态仰角+10,姿态仰角+80]才是仰角可调范围!!!
//	 */
//	tmp =(90.0 - sac->beam->attitude.t_pitch) + sac->sub_attitude->orb_data.fusion_pitch;
////	tmp =(90.0 - beam_n_pitch) + sac->sub_attitude->orb_data.fusion_pitch;
//
//	/* 确保叠加角度后,俯仰轴的值在[-90,90] */
//	if(tmp > MAX_ANGLE_PITCH)
//		sac->beam->attitude.ned_pitch = MAX_ANGLE_PITCH;
//	else if(tmp < MIN_ANGLE_PITCH)
//		sac->beam->attitude.ned_pitch = MIN_ANGLE_PITCH;
//	else
//		sac->beam->attitude.ned_pitch = tmp;
//
//	/*
//	 * yaw
//	 * 有效值范围[-180,180]绕Z轴顺时针为正,逆时针为负
//	 * T角造成的航向偏差值,正值:表示在当前角度基础上,航向逆时针偏转n度,负值:表示在当前角度基础上,航向顺时针偏转n度.
//	 * T角为正表示当逆时针旋转,而姿态是顺时针为正
//	 *
//	 * 电机的坐标系[0,360] imu的坐标系是[-180,180]
//	 * 所以当电机BASE角度大于180的时候 需要减去360 改变为[-180,0]
//	 */
//
//	/* 这里用360.0减去T角转换后的航向角,是为了将坐标系与大地坐标系统一为顺时针为正方向!!! */
//	float tmp_beam_yaw = (360.0 - sac->beam->attitude.t_yaw) + sac->beam->attitude.BASE;
////	float tmp_beam_yaw = (360.0 - beam_n_yaw) + sac->beam->attitude.BASE;
//	tmp = sac->sub_attitude->orb_data.fusion_yaw + tmp_beam_yaw;
//
//	/* 确保叠加角度后,航向轴的值在[-180,180] */
//
//	/* T角逆时针旋转n度以后,绝对值超过出180度 */
//	if(tmp < -MAX_ANGLE_YAW){
//		/* 绝对值超过出180度 */
//		sac->beam->attitude.ned_yaw = 2 * MAX_ANGLE_YAW + tmp;
//	}
//	/* T角逆时针旋转n度以后,绝对值小于180度 */
//	else{
//		sac->beam->attitude.ned_yaw = tmp;
//	}
//
//	/*
//	 * polar[roll]
//	 */
//	/* L3的姿态就是与L2的姿态夹角相差 45.0° - t_roll */
//	float tmp_beam_roll = tmp_beam_yaw + 45 - sac->beam->attitude.t_roll;
//
//	tmp = sac->sub_attitude->orb_data.fusion_yaw + tmp_beam_roll;
//
//	/* 确保叠加角度后,航向轴的值在[-180,180] */
//
//	/* T角逆时针旋转n度以后,绝对值超过出180度 */
//	if(tmp < -MAX_ANGLE_YAW){
//
//		/* 绝对值超过出180度 */
//		sac->beam->attitude.ned_roll = 2 * MAX_ANGLE_YAW + tmp;
//	}
//
//	/* T角逆时针旋转n度以后,绝对值小于180度 */
//	else{
//		sac->beam->attitude.ned_roll = tmp;
//	}
//	/*
//	 * 拷贝射束姿态信息到uORB,待发布
//	 */
//	sac->pub_beam_attitude->msg.roll 	= sac->beam->attitude.ned_roll;			//射束融合姿态后的roll
//	sac->pub_beam_attitude->msg.pitch 	= sac->beam->attitude.ned_pitch;		//射束融合姿态后的pitch
//	sac->pub_beam_attitude->msg.yaw 	= sac->beam->attitude.ned_yaw;			//射束融合姿态后的yaw
//
//	sac->pub_beam_attitude->msg.val1 	= sac->beam->attitude.T;				//T角
//	sac->pub_beam_attitude->msg.val2 	= sac->beam->attitude.t_pitch;			//T角转换后的pitch
//	sac->pub_beam_attitude->msg.val3 	= sac->beam->attitude.t_yaw;			//T角转换后的yaw
//
//	/*
//	 * 发布射束姿态信息
//	 */
//	orb_publish(ORB_ID(beam_attitude_uorb), sac->pub_beam_attitude->pub, &(sac->pub_beam_attitude->msg));
//
//	return OK;
//}
//
int beam_convert_body_to_ned(void *psat_att_ctrl)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;

	float beam_n_roll = 0.0,beam_n_pitch = 0.0,beam_n_yaw = 0.0;
	/*
	 * 调用旋转矩阵,将天线坐标系转换到大地坐标系
	 */

	imu_rotate(0, sac->beam->attitude.t_pitch,(sac->beam->attitude.t_yaw - sac->beam->attitude.BASE) , &beam_n_roll, &beam_n_pitch, &beam_n_yaw);

	printf("0   %6.2f    %6.2f    %6.2f    %6.2f    %6.2f \n", sac->beam->attitude.t_pitch, sac->beam->attitude.t_yaw,beam_n_roll, beam_n_pitch, beam_n_yaw);

	sac->beam->attitude.ned_roll  = beam_n_roll;
	sac->beam->attitude.ned_pitch = beam_n_pitch;
	sac->beam->attitude.ned_yaw   = beam_n_yaw;

	/*
	 * 拷贝射束姿态信息到uORB,待发布
	 */
	sac->pub_beam_attitude->msg.roll 	= sac->beam->attitude.ned_roll;			//射束融合姿态后的roll
	sac->pub_beam_attitude->msg.pitch 	= sac->beam->attitude.ned_pitch;		//射束融合姿态后的pitch
	sac->pub_beam_attitude->msg.yaw 	= sac->beam->attitude.ned_yaw;			//射束融合姿态后的yaw

	sac->pub_beam_attitude->msg.val1 	= sac->beam->attitude.T;				//T角
	sac->pub_beam_attitude->msg.val2 	= sac->beam->attitude.t_pitch;			//T角转换后的pitch
	sac->pub_beam_attitude->msg.val3 	= sac->beam->attitude.t_yaw;			//T角转换后的yaw

	/*
	 * 发布射束姿态信息
	 */
	orb_publish(ORB_ID(beam_attitude_uorb), sac->pub_beam_attitude->pub, &(sac->pub_beam_attitude->msg));

	return OK;
}
