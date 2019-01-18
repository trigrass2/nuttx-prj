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

#include <imu/imu_matrix.h>
#include <DSP_Lib/arm_math.h>


//加速度滤波
static bool mag_filter_outputer_flag = false;

//2HZ
//float b_MagFilt[4] = {2.19606211225382e-4,  6.58818633676145e-4,  6.58818633676145e-4,  2.19606211225382e-4};
//float a_MagFilt[4] = {1.000000000000000, -2.748835809214676, 2.528231219142559, -0.777638560238080};

//1Hz
float b_MagFilt[4] = {2.9146e-5,  8.7439e-5,  8.7439e-5,  2.9146e-5};
float a_MagFilt[4] = {1.000000000000000, -2.8744, 2.7565, -0.8819};

//0.5hZ
//float b_MagFilt[4] = {3.7568e-06,  1.1271e-05, 1.1271e-05,  3.7568e-06};
//float a_MagFilt[4] = {1.0000, -2.9372, 2.8763, -0.9391};

Vector3f mag_filter_outputer_input[4];
Vector3f mag_filter_output[4];


// a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
//    b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
//    b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
void mag_filter( Vector3f *mag , Vector3f *mag_filter_outputer)
{
	int i;

	if(!mag_filter_outputer_flag)
	{
		mag_filter_outputer_flag = true;
		for(i=1; i<=3; i++)
		{
			memcpy(mag_filter_outputer_input[i].axis, mag->axis, sizeof(mag->axis));
			memcpy(mag_filter_output[i].axis, mag->axis, sizeof(mag->axis));
		}
		memcpy(mag_filter_outputer->axis, mag->axis, sizeof(mag->axis));
	}
	else
	{
		memcpy(mag_filter_outputer_input[0].axis, mag->axis, sizeof(mag->axis));
		for(i=0; i<3; i++)
		{
			mag_filter_outputer->axis[i] = b_MagFilt[0] * mag_filter_outputer_input[0].axis[i] +
							    b_MagFilt[1] * mag_filter_outputer_input[1].axis[i] +
							    b_MagFilt[2] * mag_filter_outputer_input[2].axis[i] +
							    b_MagFilt[3] * mag_filter_outputer_input[3].axis[i] -
							    a_MagFilt[1] * mag_filter_output[1].axis[i]    -
							    a_MagFilt[2] * mag_filter_output[2].axis[i]    -
							    a_MagFilt[3] * mag_filter_output[3].axis[i];
		}

		memcpy(mag_filter_outputer_input[3].axis, mag_filter_outputer_input[2].axis, sizeof(mag_filter_outputer_input[3].axis));
		memcpy(mag_filter_outputer_input[2].axis, mag_filter_outputer_input[1].axis, sizeof(mag_filter_outputer_input[2].axis));
		memcpy(mag_filter_outputer_input[1].axis, mag_filter_outputer_input[0].axis, sizeof(mag_filter_outputer_input[1].axis));

		memcpy(mag_filter_output[3].axis, mag_filter_output[2].axis, sizeof(mag_filter_output[3].axis));
		memcpy(mag_filter_output[2].axis, mag_filter_output[1].axis, sizeof(mag_filter_output[2].axis));
		memcpy(mag_filter_output[1].axis, mag_filter_outputer->axis, sizeof(mag_filter_output[1].axis));
	}
}

