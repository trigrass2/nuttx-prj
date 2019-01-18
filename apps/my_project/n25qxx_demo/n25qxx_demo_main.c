/****************************************************************************
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


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
 * Name: usage
 *
 * Description:
 *   application use infomations.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/
static void usage(void)
{
	printf(" n25qxx_demo_main application\n");
	printf(" <cmd> <start> start the program.\n");
	printf(" <cmd> <stop> stop the program\n");
	printf(" <cmd> <status> print the program status.\n");
	printf(" <cmd> <paramget> <parameter> get all the parameters\n");
	printf(" <cmd> <paramset> <parameter> <value> set the parameter\n");
	printf("\n");
}

/****************************************************************************
 * n25qxx_demo_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int n25qxx_demo_main(int argc, char *argv[])
#endif
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start the application.
	 */
	if (!strcmp(argv[1], "start")) {
		return OK;
	}

	/*
	 * Stop the application.
	 */
	else if (!strcmp(argv[1], "stop")) {
		
	}

	/*
	 * print the application status.
	 */
	else if (!strcmp(argv[1], "status")) {

		return OK;
	}

	/*
	 * Set parameters of the application.
	 */
	else if (!strcmp(argv[1], "paramset")) {

		return OK;
	}

	/*
	 * Get parameters of the application.
	 */
	else if (!strcmp(argv[1], "paramget")) {

		return OK;
	}

	/*
	 * Print help information.
	 */
	else{
		usage();
		return -EINVAL;
	}

	return OK;
}
