/****************************************************************************
 *  apps/my_project/satellite_attitude_control/params_storage.c
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
 * Pre-processor Definitions
 ****************************************************************************/
#define EEPROM_PATH  	"/dev/ee24xx_1"
#define STORAGE_PATH 	EEPROM_PATH
/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: params_save
 *
 * Description:
 *   save the data to storage.
 *
 * Input Parameters:
 *   src    - point of data for storge.
 *   index  - start absolute address of storge.
 *   len    - size of data
 *
 ****************************************************************************/
int params_save(void *src, int index, int len)
{
	int fd = 0;
	int ret = 0;
	int nbytes = 0;

	/*
	 * 打开设备文件
	 */
	fd = open(STORAGE_PATH, O_RDWR);
	if (fd < 0){
		printf("params_save: open /dev/ee24xx_1 failed: %d\n", fd);
		return fd;
	}

	/*
	 * 写入指定长度字符串到eeprom指定位置中.
	 */
	ret = lseek(fd, index, SEEK_SET);
	if (ret < 0){
		printf("params_save: lseek:%d failed:%d\n",index , ret);
		return ret;
	}

	nbytes = write(fd, src, len);
	if (nbytes < 0){
		printf("params_save: write failed:%d\n", nbytes);
	}

	/*
	 * 关闭系统设备资源
	 */
	ret = close(fd);
	if (ret < 0){
		printf(": close /dev/eeprom_xx24xx_1 failed: %d\n", ret);
		return ret;
	}

	return nbytes;
}

/****************************************************************************
 * Name: params_load
 *
 * Description:
 *   load the data to storage.
 *
 * Input Parameters:
 *   src    - point of data for read buffer.
 *   index  - start absolute address of storge.
 *   len    - size of data
 *
 ****************************************************************************/
int params_load(void *src, int index, int len)
{
	int fd = 0;
	int ret = 0;
	int nbytes = 0;
	/*
	 * 打开设备文件
	 */
	fd = open(STORAGE_PATH, O_RDWR);
	if (fd < 0){
		printf("params_load: open /dev/ee24xx_1 failed: %d\n", fd);
		return fd;
	}

	/*
	 * 读取指定长度字符串从eeprom指定位置中.
	 */
	ret = lseek(fd, index, SEEK_SET);
	if (ret < 0){
		printf("params_load: lseek:%d failed:%d\n",index , ret);
		return ret;
	}

	nbytes = read(fd, src, len);
	if (nbytes < 0){
		printf("params_load: read failed: %d\n", nbytes);
	}

	/*
	 * 关闭系统设备资源
	 */
	ret = close(fd);
	if (ret < 0){
		printf(": close /dev/eeprom_xx24xx_1 failed: %d\n", ret);
		return ret;
	}

	return nbytes;
}

/****************************************************************************
 * Name: params_save_default
 *
 * Description:
 *   save the dafault data to storage..
 *
 * Input Parameters:
 *   src    - point of data for read buffer.
 *   index  - start absolute address of storge.
 *   len    - size of data
 *
 ****************************************************************************/
int params_save_default(void *src, int index, int len)
{
	return 0;
}

/****************************************************************************
 * Name: params_import
 *
 * Description:
 *   import factory data to storage..
 *
 * Input Parameters:
 *   src    - point of data for read buffer.
 *   index  - start absolute address of storge.
 *   len    - size of data
 *
 ****************************************************************************/
int params_import(void *src, int index, int len)
{
	return 0;
}

