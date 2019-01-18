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


void imu_init(Imu_Data *imu)
{
	imu->dt = 0.005;
	imu->dtover2 = 0.0025;
	imu->start_raw_i = 0;
	imu->start_raw_N = 20;
	imu->start_rtk_i = 0;
	imu->start_rtk_N = 20;
	imu->miss_rtk_yaw_i = 1;
	imu->start_fusion_i = 0;
	imu->start_fusion_mag_N = 600;
	imu->start_fusion_rtk_N = 1000;
	imu->fusion_out_flag = false;
//	imu->obs_yaw_select = OBS_YAW_MAG;

	imu->bg_filter_enable = false;
	imu->ba_filter_enable = true;
	imu->bm_filter_enable = true;

	imu->bg_bia.x = -0.000020;
	imu->bg_bia.y =  0.002001;
	imu->bg_bia.z =  0.001166;

//	imu->bg_bia.x = -0.000201537522003088;
//	imu->bg_bia.y =  0.000112928835722244;
//	imu->bg_bia.z = -0.000282591514890246;

	imu->bm_s.x = 1.0855;
	imu->bm_s.y = 1.0863;
	imu->bm_s.z = 1.0875;

	imu->bm_h.x =  66.4791;
	imu->bm_h.y = -11.3213;
	imu->bm_h.z =  39.8425;

	imu->att_up = true;

	imu->ARW = 0.26;
	imu->Bs_gyro = 5.1;
	imu->VRW = 0.0029;
	imu->Bs_acc = 0.07;
	imu->var_obs_roll = 4.3560e-5;
	imu->var_obs_pitch = 4.0069e-5;
	imu->var_obs_mag_yaw = 1.5e-4;
	imu->var_obs_rtk_yaw = 1.5e-4;


	imu->initd = false;
	imu->task_created = false;
	imu->subsribe_initd = false;
	imu->first_aimed = false;
	imu->re_aimed = false;
	imu->fusion_ok = false;
	imu->Qnb_update = false;
	imu->Enb_update = false;


	imu->adis16488a_fd = -1;
	imu->adis16488a_enable = true;
	imu->adis16488a_update = false;
	imu->adis16488a_time = 4000;

	imu->hmc6343_fd = -1;
	imu->hmc6343_enable = true;
	imu->hmc6343_update = false;
	imu->hmc6343_time = 199000;

	imu->gps_enable = true;
	imu->gps_update = false;
	imu->gps_yaw_correct = 0;
	imu->gps_ok = false;
	imu->rtk_filter_enable = true;

	imu->acc_switch_low = 0.98;
	imu->acc_switch_up = 1.02;
	imu->static_i = 0;
	imu->static_N = 200;

	imu->fusion_print = FUSION_PRINT_NO;
	imu->raw_print = RAW_PRINT_NO;
}





