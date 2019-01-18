/****************************************************************************
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

#ifndef __APPS_SATELLITE_ATTITUDE_CONTROL_BEAM_CONVERT_H
#define __APPS_SATELLITE_ATTITUDE_CONTROL_BEAM_CONVERT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Public Types
 ****************************************************************************/

struct beam_convert_parms_t
{
	double L;
	double A1;
	double A2;
	double A3;
	double A4;
	double A5;
	double A6;

	double M;
	double B1;
	double B2;
	double B3;
	double B4;
	double B5;
	double B6;

	double N;
	double C1;
	double C2;
	double C3;
	double C4;
	double C5;
	double C6;
};

struct beam_convert_attitude_t
{
	float T;
	float L2_L3;
	float BASE;
	float t_roll;
	float t_pitch;
	float t_yaw;
	float body_yaw;
	float body_roll;
	float body_pitch;
	float ned_roll;
	float ned_pitch;
	float ned_yaw;
};


struct beam_attitude_t
{
	struct beam_convert_parms_t 	params;

	struct beam_convert_attitude_t	attitude;

};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: beam_convert_position_to_euler
 *
 * Description:
 *   convert motors position to euler angle.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int beam_convert_position_to_euler(void *psat_att_ctrl);

/****************************************************************************
 * Name: beam_convert_body_to_ned
 *
 * Description:
 *   convert body coordinate system ned coordinate system.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int beam_convert_body_to_ned(void *psat_att_ctrl);


/****************************************************************************
 * Name: beam_convert_beam_to_ned
 *
 * Description:
 *   convert antenna coordinate system ned coordinate system.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int beam_convert_beam_to_ned(void *psat_att_ctrl);

#endif /* __APPS_SATELLITE_ATTITUDE_CONTROL_BEAM_CONVERT_H */
