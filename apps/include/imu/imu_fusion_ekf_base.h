#ifndef  __IMU_FUSION_EKF_BASE_H_INCLUDED

#define  __IMU_FUSION_EKF_BASE_H_INCLUDED


#include <imu/imu.h>
#include <imu/imu_matrix.h>
#include <DSP_Lib/arm_math.h>

#define XK_LENGTH 7
#define ZK_LENGTH 3

typedef enum EKF_Order_e
{
	Picard_1,
	Picard_2,
	Picard_3,
	Picard_4,
	Picard_5,
	Picard_6,
	RK_1,
	RK_2,
	RK_3,
	RK_4,
	RK_5,
	RK_6,
}EKF_Order;

typedef union
{
	struct
	{
		float q0;
		float q1;
		float q2;
		float q3;

		float bg_x_noise;
		float bg_y_noise;
		float bg_z_noise;
	};
	float axis[XK_LENGTH];
} Xk_Base;

typedef union
{
	struct
	{
		float roll;
		float pitch;
		float yaw;
	};
	float axis[ZK_LENGTH];
} Zk_Base;


typedef struct EKF_Base_Data_s
{
	EKF_Order order;

	bool ekf_flag;

	float ARW;
	float Bs_gyro;
	float VRW;
	float Bs_acc;

	float sigma_q;
	float sigma_roll;
	float sigma_pitch;
	float sigma_mag_yaw;
	float sigma_rtk_yaw;

	float sigma_ddw;
	float sigma_dda;

	float sigma_w;
	float sigma_wb;

	float coeff_EKF_R_static;
	float coeff_EKF_R_sport;

	float EKF_R_yaw_static;
	float EKF_R_yaw_sport;
	float EKF_R_yaw_big;

	float EKF_F[XK_LENGTH][XK_LENGTH];		//EKF矩阵F
	float EKF_Q[XK_LENGTH][XK_LENGTH];		//EKF矩阵Q
	float EKF_H[ZK_LENGTH][XK_LENGTH];		//EKF矩阵H
	float EKF_R[ZK_LENGTH][ZK_LENGTH];		//EKF矩阵R

	float EKF_Ft[XK_LENGTH][XK_LENGTH];		//EKF矩阵F的转置矩阵
	float EKF_Ht[XK_LENGTH][ZK_LENGTH];		//EKF矩阵H的转置矩阵

	Matrix4x4 Omega;
    float Ksi[4][3];

	Xk_Base xk0;
	Xk_Base xk1;
	Xk_Base delta_xk;
	Zk_Base zk;
	Zk_Base hk;
	Zk_Base vk;

	float EKF_P0[XK_LENGTH][XK_LENGTH];
	float EKF_P1[XK_LENGTH][XK_LENGTH];
	float EKF_deltaP[XK_LENGTH][ XK_LENGTH];
	float EKF_Kk[ZK_LENGTH][ZK_LENGTH];
}EKF_Base_Data;


#endif
