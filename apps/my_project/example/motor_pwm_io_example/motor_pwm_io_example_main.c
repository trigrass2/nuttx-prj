/****************************************************************************
 * my_project/examples/motor_pwm_io_example_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/fs.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/power/elmo_db4x_pwm_io.h>

#define CHANNEL(ch) (ch)

enum 
{
	CHANNEL_1 = 0,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_MAX
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * motor_pwm_io_example_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int motor_pwm_io_example_main(int argc, char *argv[])
#endif
{
	static struct motor_pwm_io_param_s g_motor_param;
	int fd;
	int ret;

	/* Configure the characteristics of the pulse train */
	if(CHANNEL_MAX > CONFIG_PWM_NCHANNELS){
		printf("invalid channel number: lager than CONFIG_PWM_NCHANNELS: %d\n", CONFIG_PWM_NCHANNELS);
	}
	

	g_motor_param.devpath = "/dev/motor_pwm1";
	g_motor_param.frequency = 1;
	g_motor_param.channels[CHANNEL_1] = CHANNEL(1);
	g_motor_param.duties[CHANNEL_1] = 0xFFFF>>1;		//Range:[-65535,65535],CW:>=0 CCW:<0
	g_motor_param.channels[CHANNEL_2] = CHANNEL(2);
	g_motor_param.duties[CHANNEL_2] = -0xFFFF>>1; 		//Range:[-65535,65535],CW:>=0 CCW:<0
	g_motor_param.channels[CHANNEL_3] = CHANNEL(3);
	g_motor_param.duties[CHANNEL_3] = -0xFFFF>>1; 		//Range:[-65535,65535],CW:>=0 CCW:<0
	g_motor_param.initialized = true;

	//打开设备文件
	fd = open(g_motor_param.devpath, O_RDONLY);
	if (fd < 0){
		printf("motor_pwm_io_example_main: open motor_pwm1 failed: %d\n", fd);
	}

	//设置两个脉冲频率为1 占空比50%
	ret = ioctl(fd, PWRIOC_RUN_SPEED, (unsigned long)((uintptr_t)&g_motor_param));
	if (ret < 0){
		printf("motor_pwm_io_example_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", ret);
	}
	sleep(5);

	//修改通道1反向，另一个通道占空比为0%
	g_motor_param.duties[CHANNEL_1] = -(0xFFFF>>1);		//Range:[-65535,65535],CW:>=0 CCW:<0
	g_motor_param.duties[CHANNEL_2] = 0;				//Range:[-65535,65535],CW:>=0 CCW:<0
	ret = ioctl(fd, PWRIOC_RUN_SPEED, (unsigned long)((uintptr_t)&g_motor_param));
	if (ret < 0){
		printf("motor_pwm_io_example_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", ret);
	}
	sleep(5);

	//安全关闭硬件定时器
	ret = ioctl(fd, PWRIOC_STOP, NULL);
	if (ret < 0){
		printf("motor_pwm_io_example_main: ioctl(PWMIOC_SETCHARACTERISTICS) failed: %d\n", ret);
	}
	
	//关闭系统资源
	ret = close(fd);
	if (ret < 0){
		printf("motor_pwm_io_example_main: open motor_pwm1 failed: %d\n", ret);
	}

  return 0;
}


















