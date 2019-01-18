#include <nuttx/config.h>
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

#include <imu/imu_matrix.h>
#include <imu/imu.h>

bool _inited = false;

Vector3f   ba;
Vector3f   bg;
Vector3f   bm;
Quaternion Q;


bool imu_aim_by_mag(Imu_Data *imu)
{
	float normalise;
	float tr,s,s1;
	Vector3f mk, my;
	float mk1;

	memcpy(ba.axis, imu->ba.axis, sizeof(ba.axis));
	memcpy(bg.axis, imu->bg.axis, sizeof(bg.axis));
	memcpy(bm.axis, imu->bm.axis, sizeof(bm.axis));

	Vector3f_Normalize(&ba);
	memcpy(mk.axis, ba.axis, sizeof(ba.axis));


	mk1 = bm.x * mk.x + bm.y * mk.y  + bm.z * mk.z;

	bm.x = bm.x - mk1 * mk.x;
	bm.y = bm.y - mk1 * mk.y;
	bm.z = bm.z - mk1 * mk.z;

	Vector3f_Normalize(&bm);

	Vector3f_Cross_Vector3f(&ba, &bm, &my);

	//Cbn
	float R[3][3] = { {bm.x,   bm.y,  bm.z},
				      {my.x,   my.y,  my.z},
					  {ba.x,   ba.y,  ba.z} };

	tr = R[0][0] + R[1][1] + R[2][2];
	s = sqrt(tr +1);
	s1 = 0.5 / s;

	Q.q0 = 0.5 *s;
	Q.q1 = (R[2][1] - R[1][2])*s1;
	Q.q2 = (R[0][2] - R[2][0])*s1;
	Q.q3 = (R[1][0] - R[0][1])*s1;

	normalise = sqrt(Q.q0 * Q.q0 + Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3);
	Q.q0  = Q.q0 / normalise;
	Q.q1  = Q.q1 / normalise;
	Q.q2  = Q.q2 / normalise;
	Q.q3  = Q.q3 / normalise;

	if(isfinite(Q.q0)  && isfinite(Q.q1) && isfinite(Q.q2)  && isfinite(Q.q3) &&  (normalise>0.95)	&& (normalise<1.05) )
	{
		_inited = true;
		memcpy(imu->Qnb.axis, Q.axis, sizeof(Q.axis));
	}
	else
	{
		_inited = false;
	}

	return _inited;
}

bool imu_aim_by_rtk(Imu_Data *imu)
{
	float normalise;
	float tr,s,s1;
	float phi, theta, psi;

	memcpy(ba.axis, imu->ba.axis, sizeof(ba.axis));
	psi = imu->gps.yaw * DEG2RAD;

	normalise = sqrt(ba.x * ba.x + ba.y * ba.y + ba.z * ba.z);
	ba.x /= normalise;
	ba.y /= normalise;
	ba.z /= normalise;

	phi = atan2((double)(ba.y),  (double)(ba.z));
	theta = -1 * asin((double)(ba.x));

	Matrix3x3 Cbn;
	Matrix3x3 Cnb;
	EulerAng Enb;

	Enb.roll = phi;
	Enb.pitch = theta;
	Enb.yaw = psi;

	Enb_2_Cbn(&Enb, &Cbn);
	Matrix3x3_Trans(&Cbn, &Cnb);

	tr = Cnb.axis[0][0] + Cnb.axis[1][1] + Cnb.axis[2][2];
	s = sqrt(tr +1);
	s1 = 0.5 / s;

	Q.q0 = 0.5 *s;
	Q.q1 = (Cnb.axis[2][1] - Cnb.axis[1][2])*s1;
	Q.q2 = (Cnb.axis[0][2] - Cnb.axis[2][0])*s1;
	Q.q3 = (Cnb.axis[1][0] - Cnb.axis[0][1])*s1;

	normalise = sqrt(Q.q0 * Q.q0 + Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3);
	Q.q0  = Q.q0 / normalise;
	Q.q1  = Q.q1 / normalise;
	Q.q2  = Q.q2 / normalise;
	Q.q3  = Q.q3 / normalise;

	if(isfinite(Q.q0)  && isfinite(Q.q1) && isfinite(Q.q2)  && isfinite(Q.q3) &&  (normalise>0.95)	&& (normalise<1.05) )
	{
		_inited = true;
		memcpy(imu->Qnb.axis, Q.axis, sizeof(Q.axis));
	}
	else
	{
		_inited = false;
	}
	return _inited;
}


void imu_aim(Imu_Data *imu)
{

	if( (imu->obs_yaw_select == OBS_YAW_MAG) && (imu->adis16488a_update) )
	{
		imu->first_aimed = imu_aim_by_mag(imu);
	}
	else if( (imu->obs_yaw_select == OBS_YAW_RTK) && (imu->gps_update) && isfinite(imu->gps.yaw) &&
			 (imu->gps.yaw != 0) && (imu->gps.Num >=9) && (imu->start_rtk_i >= imu->start_rtk_N))
	{
		imu->first_aimed = imu_aim_by_rtk(imu);
	}
	else if(imu->obs_yaw_select == OBS_YAW_MIX)
	{
		if( (imu->gps_update) && isfinite(imu->gps.yaw) &&
			(imu->gps.yaw != 0) && (imu->gps.Num >=9) && (imu->start_rtk_i >= imu->start_rtk_N) )
		{
			imu->first_aimed = imu_aim_by_rtk(imu);
		}
		else
		{
			imu->first_aimed = imu_aim_by_mag(imu);
		}
	}

	if(imu->first_aimed)
	{
		Qnb_2_Cbn(&(imu->Qnb), &(imu->Cbn));
		Cbn_2_Enb(&(imu->Cbn), &(imu->Enb));
	}
}

