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

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>
#include <math.h>
#include <satellite_signal_control/spsa_plus_algorithm.h>
#include <satellite_signal_control/normally_distributed_table.h>

#define GRADE_YAW_MAX 	(10)
#define GRADE_YAW_MIN 	(-10)
#define GRADE_PITCH_MAX (5)
#define GRADE_PITCH_MIN (-5)
/****************************************************************************
 * Name: create_random_delta_k
 *
 * Description:
 *   create random delta value.
 *
 * Input Parameters:
 *   none.
 *
 * Returned Value:
 *   random delta value range[-1,1].
 *
 ****************************************************************************/
//static inline float create_ransdom_delta_k(void)
//	return normally_distributed_table[abs(rand())%TABLE_SIZE];
//}
static inline float create_random_delta_k(void)
{
	float tmp = normally_distributed_table[abs(rand())%TABLE_SIZE];
//	if(tmp < 0) tmp = -0.2;
//	else tmp = 0.2;
//	return tmp;

	if(tmp < 0) tmp = -1;

	if(tmp >= 0) tmp = 1;
//	else tmp = 0.2;
	return tmp;

}
/****************************************************************************
 * Name: calc_loss
 *
 * Description:
 *   calculate the loss value.
 *
 * Input Parameters:
 *   pspsa_params - pointer of SPSA algorithm structure.
 *
 * Returned Value:
 *   signal power
 *
 ****************************************************************************/
static float calc_loss(float target_yaw,float target_pitch,float yaw,float pitch,float power)
{

	/*
	*	f(x) = K_az*(AZcurrent - AZ)^2 + K_el*(ELcurrent - EL)^2 + Smax
	*/

	/*
	*	f(x) = 1 - exp(A*(120 + x))
	*	其中参数A为函数的陡度因子，此处取为-0.4，越小陡度越大
	*/
//	float a = 1 - exp(-0.35*(95.0 + spsa_params->beacon_power));
//
//	spsa_params->Soutput = a * spsa_params->beacon_power + (1 - a) * temp_Soutput;
//	spsa_params->Soutput = temp_Soutput;

    double tmpaz = yaw - target_yaw;
    double  tmpel = pitch - target_pitch;
    float Soutput = ((-30) * (tmpaz * tmpaz + tmpel * tmpel))/5400 - 60;

	return Soutput;
}

float calc_loss_plus(void *pspsa_params,float target_yaw,float target_pitch,float yaw,float pitch,float power)
{
		struct spsa_algorithm_t *spsa_params = pspsa_params;
		float temp_Soutput = 0 ,Soutput;
		/*
		*	f(x) = 1 - exp(A*(100 + x))
		*	其中参数A为函数的陡度因子，此处取为-0.3，越小陡度越大
		*/
		if(power < spsa_params->params.single_range_min) power = spsa_params->params.single_range_min;

		float a = 1 - exp(-0.3*(power - spsa_params->params.single_range_min));

		/*
		*	f(x) = p1*(X - AZ)^2 + p2*(X - AZ)+ p3
		*/
		if(spsa_params->axis == SPSA_PLUS_AXIS_PITCH)
		{
			temp_Soutput = spsa_params->params.p1*(pitch- target_pitch)*(pitch- target_pitch)\
						   + spsa_params->params.p2*(pitch - target_pitch)\
						   + spsa_params->params.p3;

		}else if(spsa_params->axis == SPSA_PLUS_AXIS_YAW){
			temp_Soutput = spsa_params->params.p1*(yaw - target_yaw)*(yaw - target_yaw)\
						   + spsa_params->params.p2*(yaw - target_yaw)\
						   + spsa_params->params.p3;

		}else if(spsa_params->axis == SPSA_PLUS_AXIS_ALL){
			temp_Soutput   = spsa_params->params.k_az*(yaw - target_yaw)*(yaw - target_yaw)\
							+ spsa_params->params.k_el*(pitch- target_pitch)*(pitch- target_pitch)\
							+ spsa_params->params.S_max;
		}

		if(temp_Soutput < spsa_params->params.single_range_min){

			temp_Soutput = spsa_params->params.single_range_min;

		}
		else if(temp_Soutput > spsa_params->params.single_range_max){

			temp_Soutput = spsa_params->params.single_range_max;

		}

		Soutput = a * power + (1 - a) * temp_Soutput;

		return Soutput;
}

/****************************************************************************
 * Name: spsa_update
 *
 * Description:
 *   update SPSA disturbance quantity(delta_k).
 *
 * Input Parameters:
 *   pspsa_params - pointer of SPSA algorithm structure.
 *   pitch		  - antenna beam current pitch.
 *   yaw		  - antenna beam current yaw.
 *   power		  - antenna beacon power.
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/
float spsa_update_delta_k(void *pspsa, float curr_pitch,float curr_yaw,float curr_power)
{
	struct spsa_algorithm_t *spsa = pspsa;

	/* 1.更新扰动前位置的信号强度 */
	spsa->signal_power.tile = curr_power;
//	spsa->signal_power.tile = calc_loss(spsa->target_position.yaw,spsa->target_position.pitch,curr_yaw,curr_pitch,curr_power);
//	spsa->signal_power.tile = calc_loss_plus(spsa,spsa->target_position.yaw,spsa->target_position.pitch,curr_yaw,curr_pitch,curr_power);

	/* 2.更新当前衰减变量 */
//	spsa->params.ak = spsa->params.a / pow((spsa->params.big_a + spsa->disturbance.k + 1),spsa->params.alpha) + 0.1;
//	spsa->params.ck = spsa->params.c;

//	spsa->params.ck = spsa->params.a * spsa->signal_power.tile + spsa->params.b;
//	spsa->params.ak = spsa->params.ck * 0.5;

//	spsa->params.ck = spsa->params.c / pow((spsa->signal_power.tile - spsa->params.single_range_min + 1),spsa->params.gamma);
//	spsa->disturbance.k++;

	spsa->params.ck = spsa->params.p1 * (spsa->signal_power.tile * spsa->signal_power.tile ) \
					 +spsa->params.p2 * spsa->signal_power.tile \
					 +spsa->params.p3;

	spsa->params.ak = spsa->params.ck * 0.5;
//	spsa->params.ak = spsa->params.a / powf((spsa->params.big_a + spsa->disturbance.k + 1),spsa->params.alpha);
//	spsa->params.ck = spsa->params.c / powf((spsa->signal_power.tile - spsa->params.single_range_min + 0.1),spsa->params.gamma);
//	spsa->disturbance.k++;

	/* 3.计算当前随机扰动量 */

	spsa->disturbance.delta_k = spsa->params.ck * create_random_delta_k();


	return spsa->disturbance.delta_k;
}

/****************************************************************************
 * Name: spsa_update
 *
 * Description:
 *   update SPSA disturbance quantity(theta_k).
 *
 * Input Parameters:
 *   pspsa_params - pointer of SPSA algorithm structure.
 *   pitch		  - antenna beam current pitch.
 *   yaw		  - antenna beam current yaw.
 *   power		  - antenna beacon power.
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/
float spsa_update_theta_k(void *pspsa, float curr_pitch,float curr_yaw,float curr_power)
{
	struct spsa_algorithm_t *spsa = pspsa;

	/* 4.更新扰动后位置的信号强度 */
	spsa->signal_power.head = curr_power;
//	spsa->signal_power.head = calc_loss(spsa->target_position.yaw,spsa->target_position.pitch,curr_yaw,curr_pitch,curr_power);
//	spsa->signal_power.head = calc_loss_plus(spsa,spsa->target_position.yaw,spsa->target_position.pitch,curr_yaw,curr_pitch,curr_power);

	/* 5.更新信号梯度,并预测下一步位置最优值 */
	float theta_k_g_k_hat = (spsa->signal_power.head - spsa->signal_power.tile)/spsa->disturbance.delta_k;

	/* 6.梯度幅度输出限制 */
	if(theta_k_g_k_hat > 10) theta_k_g_k_hat = 10;
	else if(theta_k_g_k_hat < -10) theta_k_g_k_hat = -10;

	spsa->disturbance.grade = theta_k_g_k_hat;

//	if(theta_k_g_k_hat > 3 ) spsa->disturbance.theta_k = 0.2;
//	else if(theta_k_g_k_hat < -3 ) spsa->disturbance.theta_k = -0.2;
//	else spsa->disturbance.theta_k = 0;
	if(theta_k_g_k_hat >= 0 ) spsa->disturbance.theta_k = 0.1;
	else  spsa->disturbance.theta_k = -spsa->disturbance.delta_k * 1.1;



//	spsa->disturbance.theta_k = spsa->params.ak * spsa->disturbance.delta_k;;//spsa->params.ak * theta_k_g_k_hat;

    return spsa->disturbance.theta_k;
}

/****************************************************************************
 * Name: spsa_plus_run
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
void spsa_plus_run(void *pspsa_params, float pitch, float yaw, float power, bool stable)
{
	struct spsa_algorithm_t *spsa = pspsa_params;
	
	switch(spsa->run_step){
		/* 1.更新扰动量 */
		case UPDATE_DISTRITUTION:{

			spsa->disturbance.out_put = spsa_update_delta_k(spsa,pitch,yaw,power);

			spsa->run_step = UPDATE_ESTIMATED_POSITION;

		}break;

		/* 更新估计位置量 */
		case UPDATE_ESTIMATED_POSITION:{

			spsa->disturbance.out_put = spsa_update_theta_k(spsa,pitch,yaw,power);

			spsa->run_step = UPDATE_DISTRITUTION;

		}break;
	}
}







