/****************************************************************************
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
#include <nuttx/init.h>

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>

#include <debug.h>
#include <errno.h>

#include <satellite_attitude_control/pid.h>
#include <satellite_attitude_control/fuzzy_pid.h>
#include <satellite_attitude_control/sac_beam_convert.h>
#include <satellite_attitude_control/satellite_attitude_control.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MOTOR_L1_PWM_DEAD_ZONE 0.4
#define MOTOR_L2_PWM_DEAD_ZONE 0.2
#define MOTOR_L3_PWM_DEAD_ZONE 0.1

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Name: mixer_output_linear_pid
 *
 * Description:
 *   satellite attitude control output model.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *   l1				- output to L1
 *   l2				- output to L2
 *   l3				- output to L3
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int mixer_output_linear_pid(void *psat_att_ctrl,float *l1,float *l2,float *l3)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;

	/*
	 * L2 SPEED
	 */
	if(sac->beam->attitude.T < 20.0){
		*l2 = sac->rxatt_yaw2->info.desired;
	}else{
		*l2 = sac->rxatt_yaw->info.desired;
	}

	/*
	 * L1 SPEED
	 */
	*l1 = *l2 + sac->rxatt_pitch->info.desired;

	/*
	 * L3 SPEED
	 */
	if(sac->beam->attitude.T < 35.0){
		*l3 = *l2;// + sac->rxatt_polar->info.desired;
	}else{
		*l3 = *l2;// + sac->rxatt_polar2->info.desired;
	}
	return OK;
}



/****************************************************************************
 * Name: mixer_stabilize_adrc
 *
 * Description:
 *   satellite attitude control output model.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int mixer_output_adrc(void *psat_att_ctrl,float *l1,float *l2,float *l3)
{

	return OK;
}

/****************************************************************************
 * Name: mixer_stabilize_fuzzy_pid
 *
 * Description:
 *   satellite attitude control with fuzzy PID.
 *
 * Input Parameters:
 *   psat_att_ctrl  - pointer of struct sat_att_ctrl_t.
 *
 * Returned Value:
 *   the status of program.
 *
 ****************************************************************************/
int mixer_output_fuzzy_pid(void *psat_att_ctrl,float *l1,float *l2,float *l3)
{
	struct sat_att_ctrl_t *sac = psat_att_ctrl;

	float output;

	/*
	 * L2 SPEED
	 */
	output = sac->rx_yaw->info.desired;

	//电机死区补偿
	//电机死区补偿
	if(output < 0.0 && output > -MOTOR_L2_PWM_DEAD_ZONE){
		*l2 = -MOTOR_L2_PWM_DEAD_ZONE;
	}else if(output > 0.0 && output < MOTOR_L2_PWM_DEAD_ZONE ){
		*l2 = MOTOR_L2_PWM_DEAD_ZONE;
	}else{
		*l2 = output;
	}

	if(output >  sac->rx_yaw->params.o_max)output =  sac->rx_yaw->params.o_max;
	if(output < -sac->rx_yaw->params.o_max)output = -sac->rx_yaw->params.o_max;

	/*
	 * L1 SPEED
	 */
	output = sac->rx_yaw->info.desired + sac->rx_pitch->info.desired;

	//电机死区补偿
	if(output < 0.0 && output > -MOTOR_L1_PWM_DEAD_ZONE){
		*l1 = -MOTOR_L1_PWM_DEAD_ZONE;
	}else if(output > 0.0 && output < MOTOR_L1_PWM_DEAD_ZONE ){
		*l1 = MOTOR_L1_PWM_DEAD_ZONE;
	}else{
		*l1 = output;
	}

	if(output >  sac->rx_pitch->params.o_max)output =  sac->rx_pitch->params.o_max;
	if(output < -sac->rx_pitch->params.o_max)output = -sac->rx_pitch->params.o_max;

	/*
	 * L3 SPEED
	 */

	output = sac->rx_yaw->info.desired + sac->rx_polar->info.desired;
	//电机死区补偿
	if(output < 0.0 && output > -MOTOR_L3_PWM_DEAD_ZONE){
		*l3 = -MOTOR_L3_PWM_DEAD_ZONE;
	}else if(output > 0.0 && output < MOTOR_L3_PWM_DEAD_ZONE ){
		*l3 = MOTOR_L3_PWM_DEAD_ZONE;
	}else{
		*l3 = output;
	}

	if(output >  sac->rx_polar->params.o_max)output =  sac->rx_polar->params.o_max;
	if(output < -sac->rx_polar->params.o_max)output = -sac->rx_polar->params.o_max;
	return OK;
}

