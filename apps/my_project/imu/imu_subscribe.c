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
#include <sys/ioctl.h>

#include "uORB/uorb/uORB.h"
#include "uORB/topic/adis16488_uorb.h"
#include "uORB/topic/hmc6343_uorb.h"
#include "uORB/topic/gps_data_uorb.h"

#include <nuttx/sensors/adis16488.h>
#include <nuttx/sensors/hmc6343.h>

#include <imu/imu_matrix.h>
#include <imu/imu.h>

//uorb essage
struct adis16488_uorb_s sensor_adis16488;
struct hmc6343_uorb_s   sensor_hmc6343;
struct gps_data_uorb_s  sensor_gps;

bool imu_Subsribe_init(Imu_Data *imu)
{
	int ret = -1;
	/*
	 * open the adis16488a
	 */
	imu->adis16488a_fd = -1;
	if(imu->adis16488a_enable)
	{
		imu->adis16488a_fd = open("/dev/adis16488_1",O_RDWR);
		if(imu->adis16488a_fd < 0)
		{
			syslog(LOG_ERR,"ERROR: failed to open /dev/adis16488_1:%d\n",imu->adis16488a_fd);
			return false;
		}
	}

	if(imu->adis16488a_fd > 0)
	{
		ret = ioctl(imu->adis16488a_fd, ADISIOC_MEASUREMENT_RATE_SET, imu->adis16488a_time);
		if(ret < 0){
			syslog(LOG_ERR,"ERROR: failed to set measure rate adis16488:%d\n",ret);
		}

		ret = ioctl(imu->adis16488a_fd, ADISIOC_MEASUREMENT_START, 0);
		if(ret < 0){
			syslog(LOG_ERR,"ERROR: failed to start measure adis16488:%d\n", ret);
		}
	}

	imu->hmc6343_fd  =-1;
	if(imu->hmc6343_enable)
	{
		imu->hmc6343_fd = open("/dev/hmc6343_1",O_RDWR);
		if(imu->hmc6343_fd < 0){
			syslog(LOG_ERR,"ERROR: failed to open device hmc6343:%d\n",ret);
		}
	}


	if(imu->hmc6343_fd>0)
	{
		ret = ioctl(imu->hmc6343_fd, HMC6343IOC_MEASUREMENT_RATE_SET, imu->hmc6343_time);
		if(ret < 0){
			syslog(LOG_ERR,"ERROR: failed to set measure rate adis16488:%d\n",ret);
		}

		ret = ioctl(imu->hmc6343_fd, HMC6343IOC_MEASUREMENT_START, 0);
		if(ret < 0){
			syslog(LOG_ERR,"ERROR: failed to start measure hmc6343:%d\n", ret);
		}
	}

	imu->orb_adis16488a_fd = orb_subscribe(ORB_ID(adis16488_uorb));
	imu->orb_hmc6343_fd = orb_subscribe(ORB_ID(hmc6343_uorb));
	imu->orb_gps_fd = orb_subscribe(ORB_ID(gps_data_uorb));

	return true;
}


bool imu_Subsribe_adis16488(Imu_Data *imu)
{
	imu->adis16488a_update = false;
	orb_check(imu->orb_adis16488a_fd,&imu->adis16488a_update);
	if(imu->adis16488a_update)
	{
		orb_copy(ORB_ID(adis16488_uorb), imu->orb_adis16488a_fd, &sensor_adis16488);

		if( (sensor_adis16488.acc[0] == 0) && (sensor_adis16488.acc[1] == 0) && (sensor_adis16488.acc[2] == 0)
			 &&  (sensor_adis16488.gyro[0] == 0) && (sensor_adis16488.gyro[1] == 0) && (sensor_adis16488.gyro[2] == 0)
			 &&  (sensor_adis16488.mag[0] == 0) && (sensor_adis16488.mag[1] == 0) && (sensor_adis16488.mag[2] == 0) )
		{
			imu->adis16488a_update = false;
			syslog(LOG_ERR, "imu: adis16488a data all 0! \n");
			return false;
		}

		if( imu->start_raw_i <= imu->start_raw_N)
		{
			imu->start_raw_i++;
		}

		if( imu->start_raw_i >= imu->start_raw_N)
		{
			if(sensor_adis16488.acc[2] > 0)
			{
				imu->att_up = true;
			}
			else
			{
				imu->att_up = false;
			}
		}
	}
	else
	{
		syslog(LOG_ERR, "【imu】: adis16488a data is not updated! \n");
		return false;
	}

	return true;
}

bool imu_Subsribe_hmc6343(Imu_Data *imu)
{
	imu->hmc6343_update = false;
	orb_check(imu->orb_hmc6343_fd,&imu->hmc6343_update);
	if(imu->hmc6343_update)
	{
		orb_copy(ORB_ID(hmc6343_uorb), imu->orb_hmc6343_fd, &sensor_hmc6343);
	}

	return true;
}

bool imu_Subsribe_gps(Imu_Data *imu)
{
	imu->gps_update = false;
	if(imu->gps_enable)
	{
		orb_check(imu->orb_gps_fd, &imu->gps_update);
		if(imu->gps_update)
		{
			orb_copy(ORB_ID(gps_data_uorb), imu->orb_gps_fd, &sensor_gps);
		}
	}
	return true;
}

bool imu_Subsribe(Imu_Data *imu)
{
	bool flag;
	flag = imu_Subsribe_adis16488(imu);

	imu_Subsribe_hmc6343(imu);
	imu_Subsribe_gps(imu);

	return flag;
}


bool imu_raw_adis16488a(Imu_Data *imu)
{
	if(imu->adis16488a_update)
	{
		memcpy(imu->ba.axis, sensor_adis16488.acc, sizeof(imu->ba.axis));
		memcpy(imu->bg.axis, sensor_adis16488.gyro, sizeof(imu->bg.axis));
		memcpy(imu->bm.axis, sensor_adis16488.mag, sizeof(imu->bm.axis));

		if( (imu->fusion_mode != RAW_MODE) && (imu->start_raw_i >= imu->start_raw_N) )
		{
			//陀螺仪校准
			Vector3f_Sub_Vector3f(&imu->bg, &imu->bg_bia, &imu->bg);
			Vector3f_Sub_Vector3f(&imu->bg, &imu->bg_noise, &imu->bg);

			//磁力计校准，参数通过采集数据MATLAB椭球拟合得到
			Vector3f_Add_Vector3f(&imu->bm, &imu->bm_h, &imu->bm);
			Vector3f_Mult_Vector3f(&imu->bm_s,  &imu->bm, &imu->bm);

			if(!imu->att_up)
			{
				imu->ba.y =  -imu->ba.y;
				imu->ba.z =  -imu->ba.z;
				imu->bg.y =  -imu->bg.y;
				imu->bg.z =  -imu->bg.z;
				imu->bm.y =  -imu->bm.y;
				imu->bm.z =  -imu->bm.z;
			}

			//陀螺仪滤波
			if(imu->bg_filter_enable)
			{
				gyro_filter(&(imu->bg), &(imu->bg_filter));
			}
			else
			{
				memcpy(imu->bg_filter.axis, imu->bg.axis, sizeof(imu->bg.axis));
			}

			//加速度滤波
			float tmp;
			arm_power_f32(imu->ba.axis, 3, &tmp);
			arm_sqrt_f32(tmp, &imu->ba_norm);
			if(imu->ba_filter_enable)
			{
				acc_filter(&(imu->ba), &(imu->ba_filter));
			}
			else
			{
				memcpy(imu->ba_filter.axis, imu->ba.axis, sizeof(imu->ba.axis));
			}

			//磁力计滤波
			if(imu->bm_filter_enable)
			{
				mag_filter(&(imu->bm), &(imu->bm_filter));
			}
			else
			{
				memcpy(imu->bm_filter.axis, imu->bm.axis, sizeof(imu->bm.axis));
			}
		}
	}

	return true;
}


bool imu_raw_hmc6343(Imu_Data *imu)
{
	if(imu->hmc6343_update)
	{
		orb_copy(ORB_ID(hmc6343_uorb), imu->orb_hmc6343_fd, &sensor_hmc6343);
		imu->hmc.bhead.x = 0.1 * sensor_hmc6343.heading[2];
		imu->hmc.bhead.y = 0.1 * sensor_hmc6343.heading[1];
		imu->hmc.bhead.z = 0.1 * sensor_hmc6343.heading[0];

		if(imu->hmc.bhead.z > 180)
			imu->hmc.bhead.z = imu->hmc.bhead.z - 360;
	}
	return true;
}

bool imu_raw_gps(Imu_Data *imu)
{
	if(imu->gps_update)
	{
		imu->gps.lat = 0.01 * sensor_gps.latitude;
		imu->gps.lon = 0.01 *sensor_gps.longitude;
		imu->gps.alt = sensor_gps.altitude;
		imu->gps.speed = KNOT_M * sensor_gps.speed;
		imu->gps.Num = sensor_gps.satellite_no;

		if(isfinite(sensor_gps.azimuth) &&  ( sensor_gps.azimuth > 0) && ( sensor_gps.azimuth <= 360) )
		{
			if(imu->start_rtk_i <= imu->start_rtk_N)
			{
				imu->start_rtk_i++;
			}

			if( ( sensor_gps.azimuth >=0 )  && ( sensor_gps.azimuth <180 ))
			{
				imu->gps.yaw = sensor_gps.azimuth;
			}
			else if(( sensor_gps.azimuth >=180 )  && ( sensor_gps.azimuth <=360 ))
			{
				imu->gps.yaw = (sensor_gps.azimuth - 360);
			}

			imu->gps.yaw = imu->gps.yaw - imu->gps_yaw_correct;

			if(imu->gps.yaw < -180)
			{
				imu->gps.yaw = imu->gps.yaw + 360;
			}
			else if(imu->gps.yaw >= 180)
			{
				imu->gps.yaw = imu->gps.yaw - 360;
			}

			if(imu->rtk_filter_enable)
			{
				rtk_filter(&imu->gps.yaw,  &imu->gps.yaw_filter);
			}
			else
			{
				imu->gps.yaw_filter = imu->gps.yaw;
			}
			imu->gps.yaw_last  = imu->gps.yaw_filter;

			return true;
		}
		else
		{
			imu->gps.yaw = 0;
			imu->miss_rtk_yaw_i++;
		}
	}
	else
	{
		imu->miss_rtk_yaw_i++;
		imu->gps.yaw = 0;
	}

	uint32_t N1_tmp;
	N1_tmp = (1000 / imu->adis16488a_time) * 60;
	if((imu->miss_rtk_yaw_i % N1_tmp) == 0)
	{
		syslog(LOG_ERR, "[imu]: rtk yaw missing more than %d minutes! \n", imu->miss_rtk_yaw_i / N1_tmp);
	}

	return false;
}


bool imu_rawdata(Imu_Data *imu)
{
	imu_raw_adis16488a(imu);
	imu_raw_hmc6343(imu);
	imu_raw_gps(imu);
	return true;
}







