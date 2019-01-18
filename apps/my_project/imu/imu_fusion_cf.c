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

#include <imu/imu.h>
#include <imu/imu_matrix.h>

float exInt = 0.0;
float eyInt = 0.0;
float ezInt = 0.0;

//float _w_acc  = 1;	/*加速度权重*/
//float _w_mag = 1;		/*磁力计权重*/

float  _mag_decl = 0.0;
float  _bias_max = 0.05;

static Vector3f ba;
static Vector3f bg;
static Vector3f bm;
static Quaternion Q;

static float dt;

bool imu_fusion_cf(Imu_Data *imu)
{
	float normalise = 0;
	Vector3f bh;
	Vector3f bb;
	Vector3f bt;

	Vector3f err;
	Vector3f err_acc;
	Vector3f err_mag;

	Matrix3x3 Cbn;		//b->n
	Matrix3x3 Cnb;		//n->b

	float delta2  = 0;

	Vector3f q_grav;
	Vector3f q_mag;

	memcpy(ba.axis, imu->ba.axis, sizeof(ba.axis));
	memcpy(bg.axis, imu->bg.axis, sizeof(bg.axis));
	memcpy(bm.axis, imu->bm.axis, sizeof(bm.axis));
	memcpy(Q.axis, imu->Qnb.axis, sizeof(Q.axis));
	dt = imu->dt;

	float q0Last = Q.q0;
	float q1Last = Q.q1;
	float q2Last = Q.q2;
	float q3Last = Q.q3;

	normalise = sqrt(ba.x * ba.x + ba.y * ba.y + ba.z * ba.z);
	ba.x /= normalise;
	ba.y /= normalise;
	ba.z /= normalise;

	normalise = sqrt(bm.x * bm.x + bm.y * bm.y + bm.z * bm.z);
	bm.x /= normalise;
	bm.y /= normalise;
	bm.z /= normalise;


	Qnb_2_Cbn(&Q, &Cbn);
	Matrix3x3_Trans(&Cbn, &Cnb);

	Matrix3x3_Mult_Vector3f(&Cbn, &bm, &bh);

	bb.x = sqrt(bh.x * bh.x + bh.y * bh.y);
	bb.y = 0;
	bb.z = bh.z;

	Matrix3x3_Mult_Vector3f(&Cnb, &bb, &q_mag);
	Vector3f_Cross_Vector3f(&bm, &q_mag, &err_mag);

	bt.x = 0;
	bt.y = 0;
	bt.z = 1;

	Matrix3x3_Mult_Vector3f(&Cnb, &bt, &q_grav);
	Vector3f_Cross_Vector3f(&ba, &q_grav, &err_acc);

	err.x = err_acc.x + err_mag.x;
	err.y = err_acc.y + err_mag.y;
	err.z = err_acc.z + err_mag.z;

	exInt = exInt + err.x  * dt;
	eyInt = eyInt + err.y  * dt;
	ezInt = ezInt + err.z  * dt;

	float kp = 0.2;
	float ki = 0.001;
	bg.x = bg.x + (kp * err.x + ki * exInt);
	bg.y = bg.y + (kp * err.y + ki * eyInt);
	bg.z = bg.z + (kp * err.z + ki * ezInt);

	//四阶阶龙格库塔法
	delta2 = (bg.x * bg.x + bg.y * bg.y + bg.z * bg.z) * dt * dt;
    Q.q0 = q0Last*(1 - delta2/8 + delta2*delta2/384) + (-q1Last*bg.x - q2Last*bg.y - q3Last*bg.z)*dt*(0.5 - delta2/48);
    Q.q1 = q1Last*(1 - delta2/8 + delta2*delta2/384) + ( q0Last*bg.x + q2Last*bg.z - q3Last*bg.y)*dt*(0.5 - delta2/48);
    Q.q2 = q2Last*(1 - delta2/8 + delta2*delta2/384) + ( q0Last*bg.y - q1Last*bg.z + q3Last*bg.x)*dt*(0.5 - delta2/48);
    Q.q3 = q3Last*(1 - delta2/8 + delta2*delta2/384) + ( q0Last*bg.z + q1Last*bg.y - q2Last*bg.x)*dt*(0.5 - delta2/48);

	normalise = sqrt(Q.q0 * Q.q0 + Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3);
	Q.q0 = Q.q0 / normalise;
	Q.q1 = Q.q1 / normalise;
	Q.q2 = Q.q2 / normalise;
	Q.q3 = Q.q3 / normalise;

	memcpy(imu->Qnb.axis, Q.axis, sizeof(Q.axis));

	return true;
}





