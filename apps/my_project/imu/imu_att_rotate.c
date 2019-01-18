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

#include <DSP_Lib/arm_math.h>
#include <imu/imu_rotate.h>
#include <imu/imu_matrix.h>
#include <imu/imu.h>



void imu_att_rotate(float t_w_x, float t_w_y, float t_w_z,  float *n_w_roll_1, float *n_w_pitch_1, float *n_w_yaw_1,  float *n_w_roll_2, float *n_w_pitch_2, float *n_w_yaw_2)
{




}


