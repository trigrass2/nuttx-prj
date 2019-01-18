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

#include "uORB/uorb/uORB.h"
#include "uORB/topic/imu_data_uorb.h"

#include <imu/imu.h>


struct imu_data_uorb_s   imu_data0;
orb_advert_t imu_pub0;


void imu_publish_init(void)
{
	memset(&imu_data0, 0, sizeof(imu_data0));
	imu_pub0 = orb_advertise(ORB_ID(imu_data_uorb), &imu_data0);
}


void imu_publish(Imu_Data *imu)
{
	int i;
	if(imu->Enb_update)
	{
		for(i=0; i<3; i++)
		{
			imu->att.axis[i] = imu->Enb.axis[i] * RAD2DEG;
		}

		imu_data0.fusion_roll = imu->att.roll;
		imu_data0.fusion_pitch = imu->att.pitch;
		imu_data0.fusion_yaw = imu->att.yaw;
//
//		imu_data0.gx = imu->bg.x * RAD2DEG;
//		imu_data0.gy = imu->bg.y * RAD2DEG;
//		imu_data0.gz = imu->bg.z * RAD2DEG;
//		imu_data0.gz = imu->gps.yaw_last;

		imu_data0.gx = imu->Enb_obs.roll * RAD2DEG;
		imu_data0.gy = imu->Enb_obs.pitch * RAD2DEG;
		imu_data0.gz = imu->Enb_obs.yaw * RAD2DEG;


//		imu_data0.gx = imu->hmc.bhead.x;
//		imu_data0.gy = imu->hmc.bhead.y;
//		imu_data0.gz = imu->hmc.bhead.z;

		orb_publish(ORB_ID(imu_data_uorb), imu_pub0, &imu_data0);
	}
}





