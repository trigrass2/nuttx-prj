/****************************************************************************
 * app/include/satellite_signal_control/message_handle.h
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

#ifndef __APPS_SATELLITE_SIGNAL_CONTROL_MSG_HANDLE_H
#define __APPS_SATELLITE_SIGNAL_CONTROL_MSG_HANDLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "uORB/uorb/uORB.h"
#include "uORB/topic/beacon_data_uorb.h"
#include "uORB/topic/attitude_set_point_uorb.h"
#include "uORB/topic/attitude_control_status_uorb.h"
#include "uORB/topic/satellite_point_uorb.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/
/*
 * subscribed data about beacon power and so on.
 */
struct orb_beacon_info_t
{
	bool pass_check;								/* check result */
	bool enable_check;								/* check enable*/
	int orb_fd;										/* file descriptor */
	struct beacon_data_uorb_s orb_data;				/* info of beacon */
};

/*
 * subscribed data about attitude control status.
 */
struct orb_attitude_control_status_t
{
	bool pass_check;								/* check result */
	bool enable_check;								/* check enable*/
	int orb_fd;										/* file descriptor */
	struct attitude_control_status_uorb_s orb_data;	/* info of beacon */
};

/*
 * subscribed data about satellite info.
 */
struct orb_satellite_info_t
{
	bool pass_check;								/* check result */
	bool enable_check;								/* check enable*/
	int orb_fd;										/* file descriptor */
	struct satellite_point_uorb_s orb_data;			/* info of satellite */
};

/*
 * advertise data about attitude set points
 */
struct orb_attitude_set_point_t
{
	orb_advert_t 				   		pub;		/* publish fd */
	struct attitude_set_point_uorb_s	msg;		/* command data */
};
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Name: do_orb_msg_subscribe
 *
 * Description:
 *   subscribe orb message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
int ssc_do_orb_msg_subscribe(void *priv);


/****************************************************************************
 * Name: ssc_do_orb_msg_pre_check
 *
 * Description:
 *   pre-check the subscribed message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
int ssc_do_orb_msg_pre_check(void *priv);

/****************************************************************************
 * Name: ssc_do_msg_advertise
 *
 * Description:
 *   advertise the uorb message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct.
 *
 * Returned Value:
 *   NONE.
 *
 ****************************************************************************/
int ssc_do_orb_msg_advertise(void *priv);

/****************************************************************************
 * Name: ssc_do_orb_msg_poll
 *
 * Description:
 *   poll the uorb message.
 *
 * Input Parameters:
 *   priv  - satellite attitude struct
 *
 * Returned Value:
 *   poll result.
 *
 ****************************************************************************/
int ssc_do_orb_msg_poll(void * priv);

#endif /* __APPS_SATELLITE_SIGNAL_CONTROL_MSG_HANDLE_H */
