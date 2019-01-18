#include <nuttx/lib/math.h>

#include <nuttx/config.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <nuttx/fs/fs.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include <debug.h>
#include <syslog.h>

#include <imu/imu.h>
#include <imu/imu_matrix.h>
#include <imu/imu_fusion_ekf_base.h>

static EKF_Base_Data ekf_data;
static bool initd = false;

void imu_fusion_ekf_base_init(Imu_Data *imu);
void imu_fusion_ekf_base_reinit(EKF_Base_Data *ebd, Imu_Data *imu);
bool imu_fusion_ekf_base_update(EKF_Base_Data *ebd, Imu_Data *imu);

void Omega_cal(EKF_Base_Data *ebd, Imu_Data *imu);
void Ksi_cal(EKF_Base_Data *ebd, Imu_Data *imu);

void ekf_base_cal_F(EKF_Base_Data *ebd, Imu_Data *imu);
void ekf_base_cal_Q(EKF_Base_Data *ebd, Imu_Data *imu);
bool ekf_base_predict_xk(EKF_Base_Data *ebd, Imu_Data *imu);

void ekf_base_cal_P1(EKF_Base_Data *ebd, Imu_Data *imu);
bool ekf_base_cal_Kk(EKF_Base_Data *ebd, Imu_Data *imu);

void ekf_base_cal_H(EKF_Base_Data *ebd, Imu_Data *imu);
void ekf_base_cal_R(EKF_Base_Data *ebd, Imu_Data *imu);
void ekf_base_cal_vk(EKF_Base_Data *ebd, Imu_Data *imu);

bool ekf_base_update_xk(EKF_Base_Data *ebd, Imu_Data *imu);
void ekf_base_update_P0(EKF_Base_Data *ebd, Imu_Data *imu);

void ekf_base_turnswitch(EKF_Base_Data *ebd, Imu_Data *imu);
void imu_static_est(Imu_Data *imu);

bool imu_fusion_ekf_base(Imu_Data *imu)
{
	imu_fusion_ekf_base_init(imu);

	ekf_data.ekf_flag = imu_fusion_ekf_base_update(&ekf_data, imu);
	if( (!ekf_data.ekf_flag) && (imu->re_aimed) )
	{
		imu_fusion_ekf_base_reinit(&ekf_data, imu);
		imu->re_aimed = false;
		return true;
	}

	return true;
}


void imu_fusion_ekf_base_init(Imu_Data *imu)
{
	int i;
	if(!initd)
	{
		ekf_data.order = Picard_4;

		for(i=0; i<XK_LENGTH; i++)
		{
			ekf_data.EKF_P0[i][i] = 0.01;
		}

		ekf_data.ARW = imu->ARW / 60 * PI  / 180;
		ekf_data.Bs_gyro = imu->Bs_gyro / 3600 * PI  / 180;
		ekf_data.VRW = imu->VRW / 60;
		ekf_data.Bs_acc = imu->Bs_acc * 0.001 * GRAVITY;

		ekf_data.sigma_ddw = 2 * PI / logf(2) * ekf_data.Bs_gyro * ekf_data.Bs_gyro / ekf_data.ARW ;
		ekf_data.sigma_dda =  2 * PI / logf(2) * ekf_data.Bs_acc * ekf_data.Bs_acc / ekf_data.VRW;

		ekf_data.sigma_roll = imu->var_obs_roll;
		ekf_data.sigma_pitch = imu->var_obs_pitch;
		ekf_data.sigma_mag_yaw = imu->var_obs_mag_yaw;
		ekf_data.sigma_rtk_yaw = imu->var_obs_rtk_yaw;

		ekf_data.coeff_EKF_R_static = 100;
		ekf_data.coeff_EKF_R_sport  = 1000;

		ekf_data.ekf_flag = true;

		initd = true;
	}
}

void imu_fusion_ekf_base_reinit(EKF_Base_Data *ebd, Imu_Data *imu)
{
	int i;
	for(i=0; i<XK_LENGTH; i++)
	{
		ebd->EKF_P0[i][i] = 1;
	}
	syslog(LOG_ERR, "【imu】: ERROR Quaternion(xk1) reinited!\n");
}


bool imu_fusion_ekf_base_update(EKF_Base_Data *ebd, Imu_Data *imu)
{
	//EKF第1步：计算状态转移矩阵F
	ekf_base_cal_F(ebd, imu);

	//EKF第2步：计算噪声协方差矩阵Q
	ekf_base_cal_Q(ebd, imu);

	//EKF第3步：计算状态向量估计值
	ebd->ekf_flag = ekf_base_predict_xk(ebd, imu);
 	if(!ebd->ekf_flag)
	{
		return false;
	}

	//EKF第4步：构建hk, zk, vk
	ekf_base_cal_vk(ebd, imu);
	ekf_base_turnswitch(ebd, imu);

	//EKF第5步：误差协方差矩阵的传播，从P0到P1
	ekf_base_cal_P1(ebd, imu);

	//EKF第6步：计算观测矩阵H
	ekf_base_cal_H(ebd, imu);

	//EKF第7步：计算观测噪声协方差R
	imu_static_est(imu);
	ekf_base_cal_R(ebd, imu);

	//EKF第8步：计算EKF增益矩阵Kk
	ebd->ekf_flag = ekf_base_cal_Kk(ebd, imu);
	if(!ebd->ekf_flag)
	{
		return false;
	}

	//EKF第9步：状态向量估计值的更新
	ebd->ekf_flag = ekf_base_update_xk(ebd, imu);
	if(!ebd->ekf_flag)
	{
		return false;
	}

	//EKF第10步：误差协方差矩阵的更新
	ekf_base_update_P0(ebd, imu);

	return true;
}

/*-----计算F-----*/
void ekf_base_cal_F(EKF_Base_Data *ebd, Imu_Data *imu)
{
	int i,j;

	Omega_cal(ebd, imu);
	Ksi_cal(ebd, imu);

	for(i=0; i<4; i++)
	{
		for(j=0; j<4; j++)
		{
			ebd->EKF_F[i][j] = ebd->Omega.axis[i][j];
		}
	}

	for(i=0; i<4; i++)
	{
		for(j=0; j<3; j++)
		{
			ebd->EKF_F[i][j+4]  = ebd->Ksi[i][j];
		}
	}

	for(i=4; i<XK_LENGTH; i++)
	{
		ebd->EKF_F[i][i] = 1.0;
	}

	MatrixN_Trans((float *)ebd->EKF_F, XK_LENGTH, XK_LENGTH, (float *)ebd->EKF_Ft,  XK_LENGTH, XK_LENGTH);
}

/*-----计算Q-----*/
void ekf_base_cal_Q(EKF_Base_Data *ebd, Imu_Data *imu)
{
	ebd->sigma_w = 0.5 * sqrt(imu->dt) * ebd->ARW;
	ebd->sigma_q = ebd->sigma_w * ebd->sigma_w;
	ebd->sigma_wb = ebd->sigma_ddw * ebd->sigma_ddw * ebd->Bs_gyro * ebd->Bs_gyro * imu->dt * imu->dt;
//	ebd->sigma_wb = ebd->sigma_ddw * ebd->sigma_ddw * imu->dt * imu->dt;

	float Ksi_x_KsiT[4][4];
	float Ksi_x_KsiT_Coeff[4][4];
	float Ksi_T[3][4];

	MatrixN_Trans((float *)ebd->Ksi, 4, 3, (float *)Ksi_T, 3, 4);

	MatrixN_Mult_MatrixN((float *)ebd->Ksi, 4, 3,
							 (float *)Ksi_T, 3, 4,
							 (float *)Ksi_x_KsiT, 4, 4);

	float tmp;
	tmp  = ebd->sigma_w * ebd->sigma_w;
	MatrixN_Mult_Coeff((float *)Ksi_x_KsiT, 4, 4,
							    tmp,
					   (float *)Ksi_x_KsiT_Coeff, 4, 4);

	int i,j;
	for(i=0; i<4; i++)
	{
		for(j=0; j<4; j++)
		{
			ebd->EKF_Q[i][j] = Ksi_x_KsiT_Coeff[i][j];
		}
	}

	ebd->EKF_Q[4][4] = ebd->sigma_wb;
	ebd->EKF_Q[5][5] = ebd->sigma_wb;
	ebd->EKF_Q[6][6] = ebd->sigma_wb;
}

/*-----计算P1-----*/
void ekf_base_cal_P1(EKF_Base_Data *ebd, Imu_Data *imu)
{
	float FxP0[XK_LENGTH][XK_LENGTH], FxP0xFt[XK_LENGTH][XK_LENGTH];

	MatrixN_Mult_MatrixN((float *)ebd->EKF_F, XK_LENGTH, XK_LENGTH,
						 (float *)ebd->EKF_P0, XK_LENGTH, XK_LENGTH,
						 (float *)FxP0, XK_LENGTH, XK_LENGTH);

	MatrixN_Mult_MatrixN((float *)FxP0, XK_LENGTH, XK_LENGTH,
						 (float *)ebd->EKF_Ft, XK_LENGTH, XK_LENGTH,
						 (float *)FxP0xFt, XK_LENGTH, XK_LENGTH);

	MatrixN_Add_MatrixN((float *)FxP0xFt, XK_LENGTH, XK_LENGTH,
						 (float *)ebd->EKF_Q, XK_LENGTH, XK_LENGTH,
						 (float *)ebd->EKF_P1, XK_LENGTH, XK_LENGTH);
}

/*-----计算xk-----*/
bool ekf_base_predict_xk(EKF_Base_Data *ebd, Imu_Data *imu)
{
	//四元数预估值
	MatrixN_Mult_MatrixN((float *)ebd->Omega.axis, 4, 4,
						 (float *)imu->Qnb.axis, 4, 1,
						 (float *)imu->Qnb_pre.axis, 4, 1);

	//四元数预估值归一化
	Quaternion_Normalize(&(imu->Qnb_pre));

	if(isfinite(imu->Qnb_pre.q0)  && isfinite(imu->Qnb_pre.q1) && isfinite(imu->Qnb_pre.q2)  && isfinite(imu->Qnb_pre.q3) )
	{
		//生成预估xk1
		memcpy(&ebd->xk1.axis[0], imu->Qnb_pre.axis, sizeof(imu->Qnb_pre.axis));
		memcpy(&ebd->xk1.axis[4], imu->bg_bia.axis,  sizeof(imu->bg_bia.axis));
	}
	else
	{
		syslog(LOG_ERR, "【imu】: ERROR Quaternion(xk1) is not finite\n");
		return false;
	}

	return true;
}

/*-----计算H-----*/
void ekf_base_cal_H(EKF_Base_Data *ebd, Imu_Data *imu)
{
	float xphi, yphi;
	float xpsi, ypsi;
	float utheta;
	float denom;
	float multiplier;

	xphi = imu->Qnb_pre.q0 * imu->Qnb_pre.q0
		 - imu->Qnb_pre.q1 * imu->Qnb_pre.q1
		 - imu->Qnb_pre.q2 * imu->Qnb_pre.q2
		 + imu->Qnb_pre.q3 * imu->Qnb_pre.q3;
	yphi = 2.0 * (imu->Qnb_pre.q2 * imu->Qnb_pre.q3 + imu->Qnb_pre.q0 * imu->Qnb_pre.q1);
	denom = yphi * yphi + xphi * xphi;
	if(denom < 1e-3)
	{
		denom = 1e-3;
	}
	multiplier = 2.0 / denom;

	ebd->EKF_H[0][0] = multiplier * (xphi *  imu->Qnb_pre.q1 - yphi * imu->Qnb_pre.q0);
	ebd->EKF_H[0][1] = multiplier * (xphi *  imu->Qnb_pre.q0 + yphi * imu->Qnb_pre.q1);
	ebd->EKF_H[0][2] = multiplier * (xphi *  imu->Qnb_pre.q3 + yphi * imu->Qnb_pre.q2);
	ebd->EKF_H[0][3] = multiplier * (xphi *  imu->Qnb_pre.q2 - yphi * imu->Qnb_pre.q3);

	utheta = 2.0 * (imu->Qnb_pre.q1 * imu->Qnb_pre.q3 - imu->Qnb_pre.q0 * imu->Qnb_pre.q2);
	denom = sqrt(1.0 - utheta * utheta);
	if(denom < 1e-3)
	{
		denom = 1e-3;
	}
	multiplier = 2.0 / denom;

	ebd->EKF_H[1][0] = multiplier * ( imu->Qnb_pre.q2);
	ebd->EKF_H[1][1] = multiplier * (-imu->Qnb_pre.q3);
	ebd->EKF_H[1][2] = multiplier * ( imu->Qnb_pre.q0);
	ebd->EKF_H[1][3] = multiplier * (-imu->Qnb_pre.q1);

	xpsi = imu->Qnb_pre.q0 * imu->Qnb_pre.q0
		 + imu->Qnb_pre.q1 * imu->Qnb_pre.q1
		 - imu->Qnb_pre.q2 * imu->Qnb_pre.q2
		 - imu->Qnb_pre.q3 * imu->Qnb_pre.q3;
	ypsi = 2.0 * (imu->Qnb_pre.q1 * imu->Qnb_pre.q2 + imu->Qnb_pre.q0 * imu->Qnb_pre.q3);
	denom = xpsi * xpsi + ypsi * ypsi;
	if(denom < 1e-3)
	{
		denom = 1e-3;
	}
	multiplier = 2.0 / denom;

	ebd->EKF_H[2][0] = multiplier * (xpsi *  imu->Qnb_pre.q3 - ypsi * imu->Qnb_pre.q0);
	ebd->EKF_H[2][1] = multiplier * (xpsi *  imu->Qnb_pre.q2 - ypsi * imu->Qnb_pre.q1);
	ebd->EKF_H[2][2] = multiplier * (xpsi *  imu->Qnb_pre.q1 + ypsi * imu->Qnb_pre.q2);
	ebd->EKF_H[2][3] = multiplier * (xpsi *  imu->Qnb_pre.q0 + ypsi * imu->Qnb_pre.q3);

	MatrixN_Trans((float *)ebd->EKF_H, ZK_LENGTH, XK_LENGTH, (float *)ebd->EKF_Ht,  XK_LENGTH, ZK_LENGTH);
}


/*-----计算R-----*/
void ekf_base_cal_R(EKF_Base_Data *ebd, Imu_Data *imu)
{
	if(imu->static_flag == true)
	{
		ebd->EKF_R[0][0] = ebd->coeff_EKF_R_static * ebd->sigma_roll;
		ebd->EKF_R[1][1] = ebd->coeff_EKF_R_static * ebd->sigma_pitch;
		ebd->EKF_R[2][2] = ebd->coeff_EKF_R_static * ebd->sigma_mag_yaw;
	}
	else
	{
		ebd->EKF_R[0][0] = ebd->coeff_EKF_R_sport * ebd->sigma_roll;
		ebd->EKF_R[1][1] = ebd->coeff_EKF_R_sport * ebd->sigma_pitch;
		ebd->EKF_R[2][2] = ebd->coeff_EKF_R_sport * ebd->sigma_mag_yaw;
	}

	if( (ebd->hk.roll > PI/18)  || (ebd->hk.pitch > PI/18) )
	{
		ebd->EKF_R[2][2] = 2.0 * ebd->coeff_EKF_R_sport * ebd->sigma_mag_yaw;
	}

	if( (imu->obs_yaw_select == OBS_YAW_RTK) && (imu->gps_update) && (imu->gps.yaw != 0) )
	{
		ebd->EKF_R[2][2] = ebd->sigma_rtk_yaw;
	}

	float mult;
	float pitchSq = ebd->hk.pitch * ebd->hk.pitch;
	mult = (float)(1.0 + 0.65 * pitchSq);
	ebd->EKF_R[0][0] = mult * mult * ebd->EKF_R[0][0];
}

/*-----计算Kk-----*/
bool ekf_base_cal_Kk(EKF_Base_Data *ebd, Imu_Data *imu)
{
	float P1Ht[ZK_LENGTH][XK_LENGTH];
	float HxP1xHt[ZK_LENGTH][ZK_LENGTH];
	float Sk[ZK_LENGTH][ZK_LENGTH];
	float Sk_1[ZK_LENGTH][ZK_LENGTH];

	bool flag;

	MatrixN_Mult_MatrixN((float *)ebd->EKF_P0, XK_LENGTH, XK_LENGTH,
					     (float *)ebd->EKF_Ht, XK_LENGTH, ZK_LENGTH,
					     (float *)P1Ht, XK_LENGTH, ZK_LENGTH);

	MatrixN_Mult_MatrixN((float *)ebd->EKF_H, ZK_LENGTH, XK_LENGTH,
						 (float *)P1Ht, XK_LENGTH, ZK_LENGTH,
						 (float *)HxP1xHt, ZK_LENGTH, ZK_LENGTH);

	MatrixN_Add_MatrixN((float *)HxP1xHt, ZK_LENGTH, ZK_LENGTH,
						(float *)ebd->EKF_R, ZK_LENGTH, ZK_LENGTH,
					    (float *)Sk, ZK_LENGTH, ZK_LENGTH);

	flag = MatrixN_Inverse((float *)Sk, ZK_LENGTH, ZK_LENGTH,
				    (float *)Sk_1, ZK_LENGTH, ZK_LENGTH);

	if(!flag)
	{
		syslog(LOG_ERR, "【imu】: ERROR IMU EKF Base fusion inverses failed!\n");
		return false;
	}

	MatrixN_Mult_MatrixN((float *)P1Ht, XK_LENGTH, ZK_LENGTH,
					     (float *)Sk_1, ZK_LENGTH, ZK_LENGTH,
						 (float *)ebd->EKF_Kk, XK_LENGTH, ZK_LENGTH);

	return true;
}

/*-----更新Xk-----*/
bool ekf_base_update_xk(EKF_Base_Data *ebd, Imu_Data *imu)
{
	MatrixN_Mult_MatrixN((float *)ebd->EKF_Kk, XK_LENGTH, ZK_LENGTH,
						 (float *)ebd->vk.axis, ZK_LENGTH, 1,
					     (float *)ebd->delta_xk.axis, XK_LENGTH, 1);

	MatrixN_Add_MatrixN((float *)ebd->xk1.axis, XK_LENGTH, 1,
						(float *)ebd->delta_xk.axis, XK_LENGTH, 1,
						(float *)ebd->xk0.axis, XK_LENGTH, 1);

	if(isfinite(ebd->xk0.q0)  && isfinite(ebd->xk0.q1) && isfinite(ebd->xk0.q2)  && isfinite(ebd->xk0.q3) )
	{
		memcpy(imu->Qnb.axis, &(ebd->xk0.axis[0]), sizeof(imu->Qnb.axis));
		memcpy(imu->bg_noise.axis, &(ebd->xk0.axis[4]), sizeof(imu->bg_noise.axis));
		Quaternion_Normalize(&(imu->Qnb));
	}
	else
	{
		syslog(LOG_ERR, "【imu】: ERROR Quaternion(xk0) is not finite\n");
		return false;
	}

	return true;
}

/*-----更新P0-----*/
void ekf_base_update_P0(EKF_Base_Data *ebd, Imu_Data *imu)
{
	float KkxH[XK_LENGTH][XK_LENGTH];
	MatrixN_Mult_MatrixN((float *)ebd->EKF_Kk, XK_LENGTH, ZK_LENGTH,
						 (float *)ebd->EKF_H, ZK_LENGTH, XK_LENGTH,
						 (float *)KkxH, XK_LENGTH, XK_LENGTH);

	MatrixN_Mult_MatrixN((float *)KkxH, XK_LENGTH, XK_LENGTH,
						 (float *)ebd->EKF_P1, XK_LENGTH, XK_LENGTH,
						 (float *)ebd->EKF_deltaP, XK_LENGTH, XK_LENGTH);

	MatrixN_Sub_MatrixN((float *)ebd->EKF_P1, XK_LENGTH, XK_LENGTH,
						(float *)ebd->EKF_deltaP, XK_LENGTH, XK_LENGTH,
						(float *)ebd->EKF_P0, XK_LENGTH, XK_LENGTH);
}

/*-----计算vk-----*/
void ekf_base_cal_vk(EKF_Base_Data *ebd, Imu_Data *imu)
{
	//计算hk
	Qnb_2_Cbn(&(imu->Qnb_pre), &(imu->Cbn_pre));
	Cbn_2_Enb(&(imu->Cbn_pre), &(imu->Enb_pre));
	memcpy(&(ebd->hk.axis), &(imu->Enb_pre.axis), sizeof(imu->Enb_pre.axis));

	//计算zk
	Vector3f_Normalize(&(imu->ba_filter));
	imu->Enb_obs.roll = atan2((double)(imu->ba_filter.y),  (double)(imu->ba_filter.z));
	imu->Enb_obs.pitch = -1 * asin((double)(imu->ba_filter.x));

	if(imu->obs_yaw_select == OBS_YAW_MAG)
	{
		Enb_2_Cbd(&(imu->Enb), &(imu->Cbd));
		Matrix3x3_Mult_Vector3f(&(imu->Cbd), &(imu->bm_filter), &(imu->dm));
		imu->Enb_obs.yaw = atan2((double)(-imu->dm.y) , (double)(imu->dm.x));
	}
	else if(imu->obs_yaw_select == OBS_YAW_RTK)
	{
		if( (imu->gps_enable) &&(imu->gps_update) && (isfinite(imu->gps.yaw))  && (imu->gps.yaw != 0)  && (isfinite(imu->gps.Num))  && (imu->gps.Num >= 9))
		{
			imu->Enb_obs.yaw = imu->gps.yaw  * DEG2RAD;
		}
		else
		{
			imu->Enb_obs.yaw = imu->Enb_pre.yaw;
		}
	}
	else if(imu->obs_yaw_select == OBS_YAW_MIX)
	{
		if( (imu->gps_enable) &&(imu->gps_update) && (isfinite(imu->gps.yaw))  && (imu->gps.yaw != 0)  && (isfinite(imu->gps.Num))  && (imu->gps.Num >= 9))
		{
			imu->Enb_obs.yaw = imu->gps.yaw  * DEG2RAD;
		}
		else
		{
			imu->Enb_obs.yaw = imu->Enb_pre.yaw;
		}
	}

	memcpy(&(ebd->zk.axis), &(imu->Enb_obs.axis), sizeof(imu->Enb_obs.axis));

	//计算vk
	MatrixN_Sub_MatrixN((float *)ebd->zk.axis, ZK_LENGTH, 1, (float *)ebd->hk.axis, ZK_LENGTH, 1, (float *)ebd->vk.axis, ZK_LENGTH, 1);

//	ebd->vk.roll = 0;
//	ebd->vk.pitch = 0;
//	ebd->vk.yaw = 0;

	int i;
	for(i=0; i<3; i++)
	{
		if(ebd->vk.axis[i] > PI)
			ebd->vk.axis[i] = ebd->vk.axis[i] - PI*2;
		else if(ebd->vk.axis[i] < -PI)
			ebd->vk.axis[i] = ebd->vk.axis[i] + PI*2;
	}

	for(i=0; i<3; i++)
	{
		if(ebd->vk.axis[i] > PI/3)
			ebd->vk.axis[i] = PI/3;
		else if(ebd->vk.axis[i] < -PI/3)
			ebd->vk.axis[i] = -PI/3;
	}
}

/*-----修正vk-----*/
void ekf_base_turnswitch(EKF_Base_Data *ebd, Imu_Data *imu)
{
	FirstOrderLowPass(&(imu->bg_filter.z), imu->bg.z);

	float absyaw = abs(imu->bg_filter.z);
	float TILT_YAW_SWITCH_GAIN = 0.05;
	float linInterpSF = 5.4444;
	float minSwitch = 0.1745;
	float maxSwitch = 0.3490;
	float turnSwitchScaleFactor;

	if(absyaw > minSwitch)
	{
		if(absyaw < maxSwitch)
		{
			turnSwitchScaleFactor = linInterpSF * (maxSwitch - minSwitch);
		}
		else
		{
			turnSwitchScaleFactor = 0;
		}
		turnSwitchScaleFactor = TILT_YAW_SWITCH_GAIN + turnSwitchScaleFactor;
	}
	else
	{
		turnSwitchScaleFactor = 1;
	}

	ebd->vk.roll = turnSwitchScaleFactor * ebd->vk.roll;
	ebd->vk.pitch = turnSwitchScaleFactor * ebd->vk.pitch;
}

void Omega_cal(EKF_Base_Data *ebd, Imu_Data *imu)
{
	Vector3f_Mult_Coeff(&imu->bg_filter,  imu->dt, &imu->delta);

	arm_power_f32(imu->delta.axis, 3, &(imu->_2_delta0));
	imu->_4_delta0 = imu->_2_delta0 * imu->_2_delta0;

	float MD[4][4] = { {       0     , -imu->delta.x, -imu->delta.y, -imu->delta.z },
					   { imu->delta.x,        0     ,  imu->delta.z, -imu->delta.y },
					   { imu->delta.y, -imu->delta.z,        0     ,  imu->delta.x },
					   { imu->delta.z,  imu->delta.y, -imu->delta.x,        0      }  };

	memcpy(imu->Picard_MD0.axis, MD, sizeof(MD));
	Matrix4x4_One(&imu->Mone);

	if(ebd->order == Picard_1)
	{
		imu->coeff_1 = 1;
		imu->coeff_2 = 0.5;
	}
	else
	{
		imu->coeff_1 = 1.0 - 1.0/8 * imu->_2_delta0 + 1.0/384 * imu->_4_delta0;
		imu->coeff_2 = 0.5 - 1.0/48 * imu->_2_delta0;
	}

	Matrix4x4 MD1, MD2;
	Matrix4x4_Mult_Coeff(&imu->Mone, imu->coeff_1,   &MD1);
	Matrix4x4_Mult_Coeff(&imu->Picard_MD0,  imu->coeff_2,   &MD2);
	Matrix4x4_Add_Matrix4x4(&MD1, &MD2,   &ebd->Omega);
}


/*-----计算Ksi，用于F-----*/
void Ksi_cal(EKF_Base_Data *ebd, Imu_Data *imu)
{
	ebd->Ksi[0][0] = -imu->Qnb.q1;
	ebd->Ksi[0][1] = -imu->Qnb.q2;
	ebd->Ksi[0][2] = -imu->Qnb.q3;

	ebd->Ksi[1][0] =  imu->Qnb.q0;
	ebd->Ksi[1][1] = -imu->Qnb.q3;
	ebd->Ksi[1][2] =  imu->Qnb.q2;

	ebd->Ksi[2][0] =  imu->Qnb.q3;
	ebd->Ksi[2][1] =  imu->Qnb.q0;
	ebd->Ksi[2][2] = -imu->Qnb.q1;

	ebd->Ksi[3][0] = -imu->Qnb.q2;
	ebd->Ksi[3][1] =  imu->Qnb.q1;
	ebd->Ksi[3][2] =  imu->Qnb.q0;

	if(ebd->order == Picard_1)
	{
		MatrixN_Mult_Coeff((float *)ebd->Ksi, 4,  3,
							-imu->dtover2,
							(float *)ebd->Ksi, 4, 3);
	}
	else
	{
		float tmp;
		tmp = -(1.0/2 - 1.0/48 * imu->_2_delta0) * imu->dt;
		MatrixN_Mult_Coeff((float *)ebd->Ksi, 4,  3,
									tmp,
									(float *)ebd->Ksi, 4, 3);
	}
}


/*-----IMU静止状态判断-----*/
void imu_static_est(Imu_Data *imu)
{
	if( (imu->ba_norm > imu->acc_switch_low) &&  (imu->ba_norm < imu->acc_switch_up) )
	{
		if(imu->static_i <= imu->static_N)
		{
			imu->static_i++;
		}

		if(imu->static_i >= imu->static_N)
		{
			imu->static_flag = true;
		}
	}
	else
	{
		imu->static_i = 0;
		imu->static_flag = false;
	}
}






