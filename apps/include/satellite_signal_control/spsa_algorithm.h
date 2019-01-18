/******************************************************************************
 *
 *   Copyright (C) 2017-2018 RAF Research LLC. All rights reserved.
 *   Author: Bob Feretich <bob.feretich@rafresearch.com>
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
 ******************************************************************************/

#ifndef __INCLUDE_SPSA_ALGORITHM_H
#define __INCLUDE_SPSA_ALGORITHM_H

//mode
enum spsa_mode
{

	PUBLISH_DISTRITUTION = 0,
	WAITE_DISTRITUTION_STABLE,
	UPDATE_NEXT_POSITION,
	WAITE_NEXT_POSITION_STABLE,
	
};

enum{
	AXIS_YAW = 0,
	AXIS_PITCH,
};


//data
struct spsa_params_t
{
	int	  mode;
	bool  should_exit;
	bool  initialize;
	long  k;
	float theta_k[2];    //2R*1C,R1 is yaw,R2 is pitch.
	float theta_k_next[2];
	float a;
	float c;
	float big_a;
	float alpha;
	float gamma;

	float k_az; //S function'parameter
	float k_el;	//S function'parameter
	
	float vc;
	float base;

	float ak;
	float ck;
	
	float theta_plus[2];
	float theta_minus[2];
	
	float delta_k[2]; 		//random disturbance vector
	float theta_k_g_k_hat[2]; //the gradient of theta_k
	
	float beacon_power;          //dBm
	float az; //target position
	float el;
	float Smax;
	float Soutput;
	float last_Soutput;
};


/****************************************************************************
 * Name: spsa_run
 *
 * Description:
 *   calculate the loss value.
 *
 * Input Parameters:
 *   pspsa_params - pointer of SPSA algorithm structure.
 *   pitch		  - antenna beam current pitch.
 *   yaw		  - antenna beam current yaw.
 *   power		  - antenna beacon power.
 *   stable       - antenna attitude control status.
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/
void spsa_run(void *pspsa_params, float pitch, float yaw, float power, bool stable);

#endif /* __INCLUDE_SPSA_ALGORITHM_H */

