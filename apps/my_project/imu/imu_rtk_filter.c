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

#include <imu/imu.h>
#include <imu/imu_matrix.h>


//rtk滤波
static bool rtk_filter_outputer_flag = false;

//100 - 20HZ
float b_rtkFilt[4] = { 0.098531160923927, 0.295593482771781,  0.295593482771781,  0.098531160923927};
float a_rtkFilt[4] = {1.000000000000000, -0.577240524806303,  0.421787048689562, -0.056297236491843};

float rtk_filter_input[4];
float rtk_filter_output[4];

float rtk;
float rtk_filter_outputer;


// a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
//    b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
//    b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
void rtk_filter( float *rtk_in, float *rtk_out)
{
	int i;

	rtk = *rtk_in;

	if(!rtk_filter_outputer_flag)
	{
		rtk_filter_outputer_flag = true;
		for(i=1; i<=3; i++)
		{
			rtk_filter_input[i] = rtk;
			rtk_filter_output[i] = rtk;
		}
		rtk_filter_outputer = rtk;
	}
	else
	{
		rtk_filter_input[0] = rtk;
		for(i=0; i<3; i++)
		{
			rtk_filter_outputer = b_rtkFilt[0] * rtk_filter_input[0] +
							    b_rtkFilt[1] * rtk_filter_input[1] +
							    b_rtkFilt[2] * rtk_filter_input[2] +
							    b_rtkFilt[3] * rtk_filter_input[3] -
							    a_rtkFilt[1] * rtk_filter_output[1]    -
							    a_rtkFilt[2] * rtk_filter_output[2]   -
							    a_rtkFilt[3] * rtk_filter_output[3];
		}

		rtk_filter_input[3] = rtk_filter_input[2];
		rtk_filter_input[2] = rtk_filter_input[1];
		rtk_filter_input[1] = rtk_filter_input[0];

		rtk_filter_output[3] = rtk_filter_output[2];
		rtk_filter_output[2] = rtk_filter_output[1];
		rtk_filter_output[1] = rtk_filter_outputer;
	}

	*rtk_out = rtk_filter_outputer;
}

