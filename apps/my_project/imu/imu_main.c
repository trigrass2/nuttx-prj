/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/lib/math.h>
#include <nuttx/config.h>
#include <nuttx/timers/drv_hrt.h>
#include <nuttx/sensors/adis16488.h>
#include <nuttx/sensors/hmc6343.h>
#include <nuttx/fs/fs.h>

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <DSP_Lib/arm_math.h>
#include <sys/ioctl.h>

#include <syslog.h>

#include <nuttx/clock.h>

#include <imu/imu_matrix.h>
#include <imu/imu.h>

#include <imu/imu_att_rotate.h>
#include <imu/imu_rotate.h>

struct imu_arg_s{
	bool should_exit;
	pid_t pid;
};

struct imu_arg_s g_imu = {
	.should_exit = false,
	.pid = 0
};

Imu_Data imu;
static hrt_abstime abslast_t_adis;

void imu_task_init(void);
bool imu_task_update(void);

int imu_fusion_task(int argc, FAR char *argv[])
{
	if(!imu.initd)
	{
		imu_task_init();
	}

	struct pollfd sensor_fds[] =
	{
		{ .fd = imu.orb_adis16488a_fd,	.events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	while(!g_imu.should_exit)
	{
		/* poll all subscribed uorb message */
		int poll_ret = poll(sensor_fds, 1, 500);
		hrt_abstime abscurr = hrt_absolute_time();
		int etime = abscurr - abslast_t_adis;
		abslast_t_adis = abscurr;
		imu.time_pre = abscurr;

		imu.dt = etime * 0.000001;
		if(imu.dt > 0.000001 * imu.adis16488a_time)
		{
			imu.dt = 0.000001 * imu.adis16488a_time;
		}

		imu.dtover2 = imu.dt / 2;

		if (poll_ret < 0)
		{
			/* use a counter to prevent flooding (and slowing us down) */
			syslog(LOG_ERR, "imu: ERROR return value from poll(): %d, poll time: %d\n", poll_ret, 2);
		}
		else
		{

			imu_task_update();
		}
	}

	return OK;
}


void imu_task_init(void)
{
	imu_init(&imu);

	imu_Subsribe_init(&imu);
	imu_publish_init();
}


bool imu_task_update(void)
{
	//订阅数据
	imu_Subsribe(&imu);

	//数据处理，包括坐标系统一，滤波
	imu_rawdata(&imu);

	if(imu.fusion_mode == RAW_MODE)
	{
		imu_raw_print(&imu);
		return true;
	}

	//初始对准
	if(!(imu.first_aimed))
	{
		imu_aim(&imu);
		return imu.first_aimed;
	}

	//融合(若已经初始对准，则进行融合)
	if(imu.first_aimed)
	{
		bool flag;
		flag = imu_fusion(&imu);
		if(!flag)
		{
			imu.first_aimed = false;
			syslog(LOG_ERR, "【imu】: IMU fusion failed!  Will realignment ! \n");
			return false;
		}
	}

	//发布
	imu_publish(&imu);

	//数据打印(可关闭)
	imu_fusion_print(&imu);

	return true;
}



/****************************************************************************
 * CF_main
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int imu_main(int argc, char *argv[])
#endif
{
    if( '-' != *argv[1]){
    	exit(EXIT_FAILURE);
    }

	switch (*(++argv[1])){
		case 'q': /* Quit imu */
		{
			g_imu.should_exit = true;
		}
		break;

		case 'e': /* Enable ekf */
		{
			g_imu.should_exit = false;
			if(!imu.task_created)
			{
				g_imu.pid = task_create("imu fusion", 200,  8192, imu_fusion_task, NULL);
				if (g_imu.pid < 0)
				{
					int errcode = errno;
					fprintf(stderr, "ERROR: Failed to start imu fusion task: %d\n",errcode);
					return -errcode;
				}
				else
				{
					imu.task_created = true;
					imu.fusion_mode = EKF_BASE_MODE;
				}

				if(argc == 2)
				{
					imu.last_obs_yaw_select = imu.obs_yaw_select;
					imu.obs_yaw_select = OBS_YAW_MAG;
				}
				else if(argc == 3)
				{
					if(!strcmp(argv[2], "mag"))
					{
						imu.last_obs_yaw_select = imu.obs_yaw_select;
						imu.obs_yaw_select = OBS_YAW_MAG;
					}
					else if(!strcmp(argv[2], "rtk"))
					{
						imu.last_obs_yaw_select = imu.obs_yaw_select;
						imu.obs_yaw_select = OBS_YAW_RTK;
					}
					else
					{
						imu.last_obs_yaw_select = imu.obs_yaw_select;
						imu.obs_yaw_select = OBS_YAW_MAG;
					}
				}

				if(imu.obs_yaw_select != imu.last_obs_yaw_select)
				{
					imu.first_aimed = false;
					imu.re_aimed = true;
				}

			}
			else
			{
				imu.fusion_mode = EKF_BASE_MODE;
			}
		}
		break;

		case 'g': /* Enable ekf */
		{
			g_imu.should_exit = false;
			if(!imu.task_created)
			{
				g_imu.pid = task_create("imu fusion", 200,  8192, imu_fusion_task, NULL);
				if (g_imu.pid < 0)
				{
					int errcode = errno;
					fprintf(stderr, "ERROR: Failed to start imu fusion task: %d\n",errcode);
					return -errcode;
				}
				else
				{
					imu.task_created = true;
					imu.fusion_mode = EKF_GPS_MODE;
				}
			}
			else
			{
				imu.fusion_mode = EKF_GPS_MODE;
			}
		}
		break;

		case 'c': /* Enable cf */
		{
			g_imu.should_exit = false;
			if(!imu.task_created)
			{
				g_imu.pid = task_create("imu fusion", 200,  8192, imu_fusion_task, NULL);
				if (g_imu.pid < 0)
				{
					int errcode = errno;
					fprintf(stderr, "ERROR: Failed to start imu fusion task: %d\n",errcode);
					return -errcode;
				}
				else
				{
					imu.task_created = true;
					imu.fusion_mode = CF_MODE;
				}
			}
			else
			{
				imu.fusion_mode = CF_MODE;
			}
		}
		break;

		case 'r': /* adis16488 raw data */
		{
			g_imu.should_exit = false;
			if(!imu.task_created)
			{
				g_imu.pid = task_create("imu fusion", 200,  8192, imu_fusion_task, NULL);
				if (g_imu.pid < 0)
				{
					int errcode = errno;
					fprintf(stderr, "ERROR: Failed to start imu fusion task: %d\n",errcode);
					return -errcode;
				}
				else
				{
					imu.task_created = true;
					imu.fusion_mode = RAW_MODE;
				}
			}
			else
			{
				imu.fusion_mode = RAW_MODE;
			}
		}
		break;

		case 's': /* Show current status */
		{
			if(g_imu.pid > 0 && !g_imu.should_exit)
			{
				printf("imu fusion task is running. \n");
			}
			else
			{
				printf("imu fusion task is stop. \n");
			}
		}
		break;

		default:

		break;
	  }
  return 0;
}


