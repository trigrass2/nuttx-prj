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

bool imu_fusion(Imu_Data *imu)
{
	imu->Qnb_update = false;
	imu->Enb_update = false;

	if(imu->fusion_mode == CF_MODE)
	{
		imu->Qnb_update = imu_fusion_cf(imu);

		if(!imu->Qnb_update)
		{
			return false;
		}

	}
	else if(imu->fusion_mode == EKF_BASE_MODE)
	{
		imu->Qnb_update = imu_fusion_ekf_base(imu);

		if(!imu->Qnb_update)
		{
			return false;
		}

		if(imu->obs_yaw_select == OBS_YAW_MAG)
		{
			if(imu->start_fusion_i <= imu->start_fusion_mag_N)
			{
				imu->start_fusion_i ++;
				imu->fusion_out_flag = false;
			}
			else
			{
				imu->fusion_out_flag = true;
			}
		}
		else if(imu->obs_yaw_select == OBS_YAW_RTK)
		{
			if(imu->start_fusion_i <= imu->start_fusion_rtk_N)
			{
				imu->start_fusion_i ++;
				imu->fusion_out_flag = false;
			}
			else
			{
				imu->fusion_out_flag = true;
			}
		}
		else if(imu->obs_yaw_select == OBS_YAW_MIX)
		{
			if(imu->start_fusion_i <= imu->start_fusion_rtk_N)
			{
				imu->start_fusion_i ++;
				imu->fusion_out_flag = false;
			}
			else
			{
				imu->fusion_out_flag = true;
			}
		}
	}
	else if(imu->fusion_mode == EKF_GPS_MODE)
	{
		;
	}
	else if(imu->fusion_mode == RAW_MODE)
	{
		imu_raw_print(imu);
	}


	if( (imu->Qnb_update) && (imu->fusion_out_flag) )
	{
		Qnb_2_Cbn(&(imu->Qnb), &(imu->Cbn));
		Cbn_2_Enb(&(imu->Cbn), &(imu->Enb));

		//磁偏角修正
		imu->declination = imu_mag_declination(31,104);
		imu->Enb_update = true;
	}

	return true;
}





