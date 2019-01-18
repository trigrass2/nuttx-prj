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

#include "uORB/topic/hmc6343_uorb.h"

extern struct hmc6343_uorb_s   sensor_hmc6343;


void imu_fusion_print(Imu_Data *imu)
{
//	imu->fusion_print = FUSION_PRINT_ATT;

	if( (imu->fusion_print == FUSION_PRINT_QNB) && (imu->Qnb_update) )
	{
		printf( "%7.4f    %7.4f    %7.4f    %7.4f \n",imu->Qnb.q0, imu->Qnb.q1, imu->Qnb.q3, imu->Qnb.q3);
	}

	else if((imu->fusion_print == FUSION_PRINT_ATT) && (imu->Enb_update) )
	{
		printf( "%7.2f    %7.2f    %7.2f    %d    \n",
			imu->att.roll, imu->att.pitch, imu->att.yaw, imu->static_flag);
	}
	else if(imu->fusion_print == FUSION_PRINT_TIME)
	{
		printf( "%7.4f \n", imu->dt);
		imu->time_now = hrt_absolute_time();
		printf( "%d      %d      %d  \n", imu->time_pre, imu->time_now, imu->time_now - imu->time_pre);
	}
	else if(imu->fusion_print == FUSION_PRINT_NO)
	{
		;
	}
	else
	{
		;
	}

}


void imu_raw_print(Imu_Data *imu)
{
	imu->raw_print = RAW_PRINT_GYRO;

	if(imu->raw_print == RAW_PRINT_ADIS16488a)
	{
		if(imu->adis16488a_update)
		{
			printf( "%7.4f    %7.4f    %7.4f    "
					"%7.4f    %7.4f     %7.4f   "
					"%7.4f    %7.4f     %7.4f   \n",
					imu->bg.x, imu->bg.y, imu->bg.z,
					imu->ba.x, imu->ba.y, imu->ba.z,
					imu->bm.x, imu->bm.y, imu->bm.z);
		}
	}
	else if(imu->raw_print == RAW_PRINT_GYRO)
	{
		if(imu->adis16488a_update)
		{
			Vector3f_Add_Vector3f(&imu->bg_sum, &imu->bg, &imu->bg_sum);
			imu->bg_i++;
			Vector3f_Mult_Coeff(&imu->bg_sum, 1.0/(imu->bg_i), &imu->bg_mean);

			printf( "%7.4f    %7.4f    %7.4f    "
						"%7.6f    %7.6f    %7.6f   \n",
						imu->bg.x, imu->bg.y, imu->bg.z,
						imu->bg_mean.x, imu->bg_mean.y, imu->bg_mean.z);
		}
	}
	else if(imu->raw_print == RAW_PRINT_HMC6343)
	{
		if(imu->hmc6343_update)
		{
			printf( "%7.4f    %7.4f     %7.4f   %7.4f    %7.4f     %7.4f   \n",
					imu->hmc.bhead.x, imu->hmc.bhead.y, imu->hmc.bhead.z,
					sensor_hmc6343.heading[0],  sensor_hmc6343.heading[1],  sensor_hmc6343.heading[2]);
		}
	}

	else if(imu->raw_print == RAW_PRINT_NO)
	{
		;
	}
	else
	{
		;
	}

}





