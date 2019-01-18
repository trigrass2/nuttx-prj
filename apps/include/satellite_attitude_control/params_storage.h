/****************************************************************************
 * app/include/satellite_attitude_control/params_storage.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_PARAMS_STORAGE_H
#define __APPS_PARAMS_STORAGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
enum
{
	ADDR_ATT_RX_CTRL_PITCH 			= 0,
	ADDR_ATT_RX_CTRL_YAW 			= 100,
	ADDR_ATT_RX_CTRL_YAW2 			= 200,
	ADDR_ATT_RX_CTRL_POLAR 			= 400,
	ADDR_ATT_RX_CTRL_POLAR2 		= 600,
	ADDR_ATT_RX_CTRL_T 				= 700,
	ADDR_ATT_RX_CTRL_FUZZY_PITCH 	= 700,
	ADDR_ATT_RX_CTRL_FUZZY_YAW 		= 800,
	ADDR_ATT_RX_CTRL_FUZZY_POLAR 	= 900,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/



/****************************************************************************
 * Public Function Prototypes
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
int params_save(void *src, int index, int len);

/****************************************************************************
 * Name: params_save
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
int params_load(void *src, int index, int len);

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
int params_save_default(void *src, int index, int len);


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
int params_import(void *src, int index, int len);

#endif /* __APPS_PARAMS_STORAGE_H */
