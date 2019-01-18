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

#include <imu/imu_matrix.h>
#include <imu/imu.h>

//加速度滤波
static bool gyro_filter_outputer_flag = false;

//100-2HZ
//float b_gyroFilt[4] = {2.19606211225382e-4,  6.58818633676145e-4,  6.58818633676145e-4,  2.19606211225382e-4};
//float a_gyroFilt[4] = {1.000000000000000, -2.748835809214676, 2.528231219142559, -0.777638560238080};

//100-20HZ
float b_gyroFilt[4] = {0.098531160923927,  0.295593482771781,  0.295593482771781,  0.098531160923927};
float a_gyroFilt[4] = {1.000000000000000, -0.577240524806303,  0.421787048689562, -0.056297236491843};

Vector3f gyro_filter_outputer_input[4];
Vector3f gyro_filter_output[4];


// a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
//    b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
//    b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
void gyro_filter( Vector3f *gyro , Vector3f *gyro_filter_outputer)
{
	int i;

	if(!gyro_filter_outputer_flag)
	{
		gyro_filter_outputer_flag = true;
		for(i=1; i<=3; i++)
		{
			memcpy(gyro_filter_outputer_input[i].axis, gyro->axis, sizeof(gyro->axis));
			memcpy(gyro_filter_output[i].axis, gyro->axis, sizeof(gyro->axis));
		}
		memcpy(gyro_filter_outputer->axis, gyro->axis, sizeof(gyro->axis));
	}
	else
	{
		memcpy(gyro_filter_outputer_input[0].axis, gyro->axis, sizeof(gyro->axis));
		for(i=0; i<3; i++)
		{
			gyro_filter_outputer->axis[i] = b_gyroFilt[0] * gyro_filter_outputer_input[0].axis[i] +
							    b_gyroFilt[1] * gyro_filter_outputer_input[1].axis[i] +
							    b_gyroFilt[2] * gyro_filter_outputer_input[2].axis[i] +
							    b_gyroFilt[3] * gyro_filter_outputer_input[3].axis[i] -
							    a_gyroFilt[1] * gyro_filter_output[1].axis[i]    -
							    a_gyroFilt[2] * gyro_filter_output[2].axis[i]    -
							    a_gyroFilt[3] * gyro_filter_output[3].axis[i];
		}

		memcpy(gyro_filter_outputer_input[3].axis, gyro_filter_outputer_input[2].axis, sizeof(gyro_filter_outputer_input[3].axis));
		memcpy(gyro_filter_outputer_input[2].axis, gyro_filter_outputer_input[1].axis, sizeof(gyro_filter_outputer_input[2].axis));
		memcpy(gyro_filter_outputer_input[1].axis, gyro_filter_outputer_input[0].axis, sizeof(gyro_filter_outputer_input[1].axis));

		memcpy(gyro_filter_output[3].axis, gyro_filter_output[2].axis, sizeof(gyro_filter_output[3].axis));
		memcpy(gyro_filter_output[2].axis, gyro_filter_output[1].axis, sizeof(gyro_filter_output[2].axis));
		memcpy(gyro_filter_output[1].axis, gyro_filter_outputer->axis, sizeof(gyro_filter_output[1].axis));
	}
}

