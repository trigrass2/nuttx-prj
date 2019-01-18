/****************************************************************************
 * my_project/examples/hmc6343_example_main.c
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

#include <nuttx/sensors/hmc6343.h>
#include <nuttx/timers/drv_hrt.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * usage
 ****************************************************************************/
static void usage(void)
{
	printf("Honeywell HMC6343 example\n");
	printf("<opt> <param> to test\n");
	printf("-v : get software version.\n");
	printf("-n : get serial numbers. \n");
	printf("-m : get magnetometer. \n");
	printf("-a : get accelerometer. \n");
	printf("-p : headind information. \n");
	printf("-t : tilt information. \n");
	printf("-h : help information. \n");
}

/****************************************************************************
 * hmc6343_example_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hmc6343_example_main(int argc, char *argv[])
#endif
{
	int fd = 0;
	int ret = 0;
	int loopcnt = 50;

	static struct hmc6343_sample_s sample_acc;
	static struct hmc6343_sample_s sample_mag;
	static struct hmc6343_sample_s sample_hed;
	static struct hmc6343_sample_s sample_tilt;

	uint8_t sw;
	uint16_t sn;

	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	usleep(500);

	//打开设备文件
	fd = open("/dev/hmc6343_1", O_RDWR);
	if (fd < 0){
		printf("hmc6343_example_main: open /dev/hmc6343_1 failed: %d\n", fd);
		return fd;
	}

	/*
	 * 读取设备序列号.
	 */
	else if (!strcmp(argv[1], "-n")){
		ret = ioctl(fd, HMC6343IOC_SOFTWARE_VERSION_GET, (long unsigned int)(&sw));
		if (ret < 0){
			printf("hmc6343_example_main: ioctl(HMC6343IOC_SOFTWARE_VERSION_GET) failed: %d\n", ret);
			return ret;
		}else{
			printf("software version:0x%x\n",sw);
		}
	}

	/*
	 * 读取设备的软件版本.
	 */
	else if (!strcmp(argv[1], "-v")){
		ret = ioctl(fd, HMC6343IOC_SERIAL_NUMBER_GET, (long unsigned int)(&sn));
		if (ret < 0){
			printf("hmc6343_example_main: ioctl(HMC6343IOC_SERIAL_NUMBER_GET) failed: %d\n", ret);
			return ret;
		}else{
			printf("serial numbers:0x%x\n",sn);
		}
	}

	else
	{
		while(loopcnt--){

			/*
			 * 读取设备x、y、z的磁力计数据.
			 */
			if (!strcmp(argv[1], "-m")){
				hrt_abstime t1 = hrt_absolute_time();
				ret = ioctl(fd, HMC6343IOC_MAGNETOMETER_GET, (long unsigned int)(&sample_mag));
				if (ret < 0){
					printf("hmc6343_example_main: ioctl(HMC6343IOC_MAGNETOMETER_GET) failed: %d\n", ret);
					return ret;
				}else{
					printf("MAG_X:%6d  MAG_Y:%6d  MAG_Z:%6d time:%d\n",sample_mag.X,sample_mag.Y,sample_mag.Z,hrt_elapsed_time(&t1));
				}

				usleep(1000*100);
			}

			/*
			 * 读取设备x、y、z的加速度计数据.
			 */
			else if (!strcmp(argv[1], "-a")){
				ret = ioctl(fd, HMC6343IOC_ACCELEROMETER_GET, (long unsigned int)(&sample_acc));
				if (ret < 0){
					printf("hmc6343_example_main: ioctl(HMC6343IOC_ACCELEROMETER_GET) failed: %d\n", ret);
					return ret;
				}else{
					printf("ACC_X:%6d  ACC_Y:%6d  ACC_Z:%6d \n",sample_acc.X,sample_acc.Y,sample_acc.Z);
				}
				usleep(1000*100);
			}

			/*
			 * 读取设备x、y、z的航向数据.
			 */
			else if (!strcmp(argv[1], "-p")){
				ret = ioctl(fd, HMC6343IOC_HEADING_DATA_GET, (long unsigned int)(&sample_hed));
				if (ret < 0){
					printf("hmc6343_example_main: ioctl(HMC6343IOC_HEADING_DATA_GET) failed: %d\n", ret);
					return ret;
				}else{
					printf("YAW:%6d  PITCH:%6d  ROLL:%6d \n",sample_hed.X,sample_hed.Y,sample_hed.Z);
				}
				usleep(1000*100);
			}
			/*
			 * 读取设备x、y、z的倾斜数据.
			 */
			else if (!strcmp(argv[1], "-t")){
				ret = ioctl(fd, HMC6343IOC_TILT_DATA_GET, (long unsigned int)(&sample_tilt));
				if (ret < 0){
					printf("hmc6343_example_main: ioctl(HMC6343IOC_TILT_DATA_GET) failed: %d\n", ret);
					return ret;
				}else{
					printf("PITCH:%6d  ROLL:%6d  TEMP:%6d \n",sample_tilt.X,sample_tilt.Y,sample_tilt.Z);
				}
				usleep(1000*100);
			}

			else
			{
				usage();
				return -EINVAL;
			}
		}
	}
	//关闭系统资源
	ret = close(fd);
	if (ret < 0){
		printf("hmc6343_example_main: close /dev/hmc6343_1 failed: %d\n", ret);
	}

  return 0;
}


















