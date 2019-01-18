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
#include <nuttx/timers/drv_hrt.h>
#include <DSP_Lib/arm_math.h>
#include <imu/imu_rotate.h>
#include <imu/imu_matrix.h>
#include <imu/imu.h>
#include <imu/imu_rotate.h>

extern Imu_Data imu;

void imu_rotate(float b_a_x, float b_a_y, float b_a_z, float *n_a_x, float *n_a_y, float *n_a_z)
{
	Matrix3x3 Cbb;
	Matrix3x3 Cnn;

	EulerAng  E1;

	E1.roll = PI;
	E1.pitch = 0;
	E1.yaw = PI/4;

	Enb_2_Cbn(&E1, &Cbb);

	Cnn.T11 =  1;
	Cnn.T22 = -1;
	Cnn.T33 = -1;

	Vector3f src;
	Vector3f des;
	Vector3f tmp1;
	Vector3f tmp2;

	src.x = b_a_x;
	src.y = b_a_y;
	src.z = b_a_z;

	Matrix3x3_Mult_Vector3f(&Cbb, &src, &tmp1);
	Matrix3x3_Mult_Vector3f(&(imu.Cbn), &tmp1, &tmp2);
	Matrix3x3_Mult_Vector3f(&Cnn, &tmp2, &des);

	*n_a_x = des.x;
	*n_a_y = des.y;
	*n_a_z = des.z;
}





