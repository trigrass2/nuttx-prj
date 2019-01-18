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
static bool acc_filter_outputer_flag = false;
float b_AccelFilt[4] = {2.19606211225382e-4,  6.58818633676145e-4,  6.58818633676145e-4,  2.19606211225382e-4};
float a_AccelFilt[4] = {1.000000000000000, -2.748835809214676, 2.528231219142559, -0.777638560238080};

Vector3f acc_filter_outputer_input[4];
Vector3f acc_filter_output[4];

void FirstOrderLowPass( float *output,  float input )
{
    static float inputPast;

    *output = (float)(0.984414127416097) * (*output) + (float)(0.007792936291952) * (input + inputPast);
    inputPast = input;
}


// a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
//    b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
//    b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
void acc_filter( Vector3f *acc , Vector3f *acc_filter_outputer)
{
	int i;

	if(!acc_filter_outputer_flag)
	{
		acc_filter_outputer_flag = true;
		for(i=1; i<=3; i++)
		{
			memcpy(acc_filter_outputer_input[i].axis, acc->axis, sizeof(acc->axis));
			memcpy(acc_filter_output[i].axis, acc->axis, sizeof(acc->axis));
		}
		memcpy(acc_filter_outputer->axis, acc->axis, sizeof(acc->axis));
	}
	else
	{
		memcpy(acc_filter_outputer_input[0].axis, acc->axis, sizeof(acc->axis));
		for(i=0; i<3; i++)
		{
			acc_filter_outputer->axis[i] = b_AccelFilt[0] * acc_filter_outputer_input[0].axis[i] +
							    b_AccelFilt[1] * acc_filter_outputer_input[1].axis[i] +
							    b_AccelFilt[2] * acc_filter_outputer_input[2].axis[i] +
							    b_AccelFilt[3] * acc_filter_outputer_input[3].axis[i] -
							    a_AccelFilt[1] * acc_filter_output[1].axis[i]    -
							    a_AccelFilt[2] * acc_filter_output[2].axis[i]    -
							    a_AccelFilt[3] * acc_filter_output[3].axis[i];
		}

		memcpy(acc_filter_outputer_input[3].axis, acc_filter_outputer_input[2].axis, sizeof(acc_filter_outputer_input[3].axis));
		memcpy(acc_filter_outputer_input[2].axis, acc_filter_outputer_input[1].axis, sizeof(acc_filter_outputer_input[2].axis));
		memcpy(acc_filter_outputer_input[1].axis, acc_filter_outputer_input[0].axis, sizeof(acc_filter_outputer_input[1].axis));

		memcpy(acc_filter_output[3].axis, acc_filter_output[2].axis, sizeof(acc_filter_output[3].axis));
		memcpy(acc_filter_output[2].axis, acc_filter_output[1].axis, sizeof(acc_filter_output[2].axis));
		memcpy(acc_filter_output[1].axis, acc_filter_outputer->axis, sizeof(acc_filter_output[1].axis));
	}
}

