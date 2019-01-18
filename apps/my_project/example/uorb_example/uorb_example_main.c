/****************************************************************************
 * examples/hello/hello_main.c
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

#include "uORB/uorb/uORB.h"
#include "uORB/topic/sensor_combined.h"

#include <DSP_Lib/arm_math.h>
#define ORB_POLL
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/

int sub_task(int argc, FAR char *argv[])
{

	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe_multi(ORB_ID(sensor_combined),0);

#ifdef ORB_CHECK

	/* obtained data for the first file descriptor */
	for(;;){

		bool update = false;

		orb_check(sensor_sub_fd,&update);
		if(update){
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
			printf("acc_x:%3.6f acc_y:%3.6f acc_z:%3.6f \n",raw.gyro_rad[0],raw.gyro_rad[1],raw.gyro_rad[2]);
		}else{
			sleep(1);
		}
	}
#endif

#ifdef ORB_POLL
	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	/* obtained data for the first file descriptor */
	for(;;){

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 500);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("Got no data within a second\n");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("ERROR return value from poll(): %d\n", poll_ret);
			}

			error_counter++;

		}else{
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
			printf("acc_x:%3.6f acc_y:%3.6f acc_z:%3.6f \n",raw.gyro_rad[0],raw.gyro_rad[1],raw.gyro_rad[2]);
		}
	}
#endif

	return OK;

}

int pub_task(int argc, FAR char *argv[])
{
	/* advertise attitude topic */
	struct sensor_combined_s att;
	memset(&att, 0, sizeof(att));

	int ins;
	orb_advert_t att_pub = orb_advertise_multi(ORB_ID(sensor_combined), &att,&ins,1);
	for(;;){

		/* set att and publish this information for other apps */
		att.gyro_rad[0] = 1.23;
		att.gyro_rad[1] = 4.56;
		att.gyro_rad[2] = 7.89;
		orb_publish(ORB_ID(sensor_combined), att_pub, &att);
		sleep(1);
	}
	return OK;
}

int pub_task2(int argc, FAR char *argv[])
{
	/* advertise attitude topic */
	struct sensor_combined_s att;
	memset(&att, 0, sizeof(att));

	int ins;
	orb_advert_t att_pub = orb_advertise_multi(ORB_ID(sensor_combined), &att,&ins,2);

	for(;;){

		/* set att and publish this information for other apps */
		att.gyro_rad[0] = 0.00;
		att.gyro_rad[1] = 1.11;
		att.gyro_rad[2] = 2.22;
		orb_publish(ORB_ID(sensor_combined), att_pub, &att);
		sleep(1);
	}
	return OK;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int uorb_example_main(int argc, char *argv[])
#endif
{
	static pid_t g_sub_pid,g_pub2_pid,g_pub_pid;

	g_pub_pid = task_create("demo publish", 100,
                  4096, pub_task,
                  NULL);
    if (g_pub_pid < 0){
        int errcode = errno;
        fprintf(stderr, "ERROR: Failed to start demo publish: %d\n",errcode);
        return -errcode;
      }

    g_pub2_pid = task_create("demo publish2", 101,
                  4096, pub_task2,
                  NULL);
    if (g_pub2_pid < 0){
        int errcode = errno;
        fprintf(stderr, "ERROR: Failed to start demo publish: %d\n",errcode);
        return -errcode;
      }

	g_sub_pid = task_create("demo subscribe", 90,
                  2048, sub_task,
                  NULL);
    if (g_sub_pid < 0){
        int errcode = errno;
        fprintf(stderr, "ERROR: Failed to start demo subscribe: %d\n",errcode);
        return -errcode;
      }

  return 0;
}


















