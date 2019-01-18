/****************************************************************************
 * my_project/examples/eeprom_xx24xx_example_main.c
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

#include <nuttx/eeprom/i2c_xx24xx.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#define LENTH (1024)

/****************************************************************************
 * usage
 ****************************************************************************/
static void usage(void)
{
	printf("eeprom_xx24xx example\n");
	printf("<opt> <param> to test\n");
	printf("-w <size>: write a string to eeprom_xx24xx.\n");
	printf("-r <size>: read a string to eeprom_xx24xx. \n");
	printf("-f <start> <size>: flush <size> eeprom_xx24xx at <start>. \n");
	printf("-h : help information. \n");
}

/****************************************************************************
 * eeprom_xx24xx_example_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int eeprom_xx24xx_example_main(int argc, char *argv[])
#endif
{
	int fd = 0;
	int ret = 0;
	int i = 0;
	static int32_t nsize = 0 ,start = 0;
	static char src_buffer[LENTH];
	static char dst_buffer[LENTH];

	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	else{
		//打开设备文件
		fd = open("/dev/ee24xx_1", O_RDWR);
		if (fd < 0){
			printf("eeprom_xx24xx_example_main: open /dev/ee24xx_1 failed: %d\n", fd);
			return fd;
		}

		/*
		 * 清除指定段eeprom.
		 */
		 if (!strcmp(argv[1], "-f")){

			start = atoi(argv[2]);
			nsize = atoi(argv[3]);

			for(i = 0; i < nsize; i++){
				src_buffer[i] = 0xFF;
			}

			ret = lseek(fd, start, SEEK_SET);
			if (ret < 0){
				printf("lseek:%d failed:%d\n",i , ret);
				return ret;
			}

			ret = write(fd, src_buffer, nsize);
			if (ret < 0){
				printf("eeprom_xx24xx_example_main: write failed:%d\n", ret);
				return ret;
			}else{
				printf("eeprom_xx24xx_example_main write[%d/%d] bytes\n", i,nsize);
			}
		}

		/*
		 * 写入指定字符串到eeprom中.
		 */
		else if (!strcmp(argv[1], "-w")){

			nsize = atoi(argv[2]);

			for(i = 0; i < nsize; i++){
				src_buffer[i] = i;
			}

			ret = lseek(fd, 0, SEEK_SET);
			if (ret < 0){
				printf("lseek:%d failed:%d\n",i , ret);
				return ret;
			}

			ret = write(fd, src_buffer, nsize);
			if (ret < 0){
				printf("eeprom_xx24xx_example_main: write failed:%d\n", ret);
				return ret;
			}else{
				printf("eeprom_xx24xx_example_main write[%d/%d] bytes\n", i,nsize);
			}
		}

		/*
		 * 读取指定字符串从eeprom中.
		 */
		else if (!strcmp(argv[1], "-r")){
			nsize = atoi(argv[2]);

			ret = lseek(fd, 0, SEEK_SET);
			if (ret < 0){
				printf("lseek:%d failed:%d\n",i , ret);
				return ret;
			}

			ret = read(fd, dst_buffer, nsize);
			if (ret < 0){
				printf("eeprom_xx24xx_example_main: read failed: %d\n", ret);
				return ret;
			}else{
				for(i = 0; i < ret; i++){
					printf(" %3d",dst_buffer[i]);
					if((i+1)%20 == 0){
						printf("\n");
					}
				}
				printf("\n");
			}
		}

		else
		{
			usage();
			return -EINVAL;
		}

		//关闭系统资源
		ret = close(fd);
		if (ret < 0){
			printf("eeprom_xx24xx_example_main: close /dev/eeprom_xx24xx_1 failed: %d\n", ret);
		}
	}
  return 0;
}


















