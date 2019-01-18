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
#include <stdbool.h>
#include <math.h>
#include <satellite_signal_control/spsa_algorithm.h>
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
static inline float create_random_delta_k(void)
{
	return normally_distributed_table[abs(rand())%TABLE_SIZE];
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
 *   none.
 *
 ****************************************************************************/
static void calc_loss(void *pspsa_params)
{
	struct spsa_params_t *spsa_params = pspsa_params;

	/*
	*	f(x) = K_az*(AZcurrent - AZ)^2 + K_el*(ELcurrent - EL)^2 + Smax
	*/
	float tmp_AZ = spsa_params->theta_k[AXIS_YAW] - spsa_params->az;
	if(tmp_AZ > GRADE_YAW_MAX) tmp_AZ = GRADE_YAW_MAX;
	if(tmp_AZ < GRADE_YAW_MIN) tmp_AZ = GRADE_YAW_MIN;

	float tmp_EL = spsa_params->theta_k[AXIS_PITCH] - spsa_params->el;
	if(tmp_EL > GRADE_PITCH_MAX) tmp_EL = GRADE_PITCH_MAX;
	if(tmp_EL < GRADE_PITCH_MIN) tmp_EL = GRADE_PITCH_MIN;

	float temp_Soutput =  spsa_params->k_az * tmp_AZ * tmp_AZ + spsa_params->k_el * tmp_EL * tmp_EL + spsa_params->Smax;

	/*
	*	f(x) = 1 - exp(A*(120 + x))
	*	其中参数A为函数的陡度因子，此处取为-0.4，越小陡度越大
	*/
	float a = 1 - exp(-0.35*(95.0 + spsa_params->beacon_power));
//	float a = 1;

	spsa_params->Soutput = a * spsa_params->beacon_power + (1 - a) * temp_Soutput;
	spsa_params->Soutput = temp_Soutput;
}

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
void spsa_run(void *pspsa_params, float pitch, float yaw, float power, bool stable)
{
	struct spsa_params_t *spsa_params = pspsa_params;
	
	//subscription beacon_power/pitch/yaw
	spsa_params->theta_k[AXIS_PITCH] = pitch;
	spsa_params->theta_k[AXIS_YAW] 	 = yaw;
	spsa_params->beacon_power  		 = power;
	static int idle_cycle = 0;
	switch(spsa_params->mode)
	{

		case WAITE_NEXT_POSITION_STABLE:
		{
			if(idle_cycle ++ > 2){
				spsa_params->mode = PUBLISH_DISTRITUTION;
			}
		}break;

		//create delta_k
		case PUBLISH_DISTRITUTION:
		{
			/* 判断位置是否稳定 */
			if(stable){
				spsa_params->delta_k[AXIS_PITCH] = create_random_delta_k();
				spsa_params->delta_k[AXIS_YAW]   = create_random_delta_k();

				//calc ak
				spsa_params->ak = spsa_params->a / pow((spsa_params->big_a + spsa_params->k + 1),spsa_params->alpha) + 0.1;

				//calc ck
				spsa_params->ck = spsa_params->c / pow((spsa_params->k + 1),spsa_params->gamma);

				//initialise gradient estimate
				spsa_params->theta_k_g_k_hat[AXIS_PITCH] = 0;
				spsa_params->theta_k_g_k_hat[AXIS_YAW] = 0;

				/* 计算扰动值 */
				spsa_params->theta_plus[AXIS_YAW]    = spsa_params->theta_k[AXIS_YAW] + spsa_params->ck * spsa_params->delta_k[AXIS_YAW];
				spsa_params->theta_plus[AXIS_PITCH]  = spsa_params->theta_k[AXIS_PITCH] + spsa_params->ck * spsa_params->delta_k[AXIS_PITCH];
				//spsa_params->theta_minus = spsa_params->theta_k - spsa_params->ck * spsa_params->delta_k;

				/* 如果信号丢失需要清除K,否则K自增加 */
				spsa_params->k++;

				/* 进入等待,等待扰动稳定 */
				spsa_params->mode = WAITE_DISTRITUTION_STABLE;
			}
		}
		break;

		case WAITE_DISTRITUTION_STABLE:
		{
			/* 判断扰动是否稳定 */
			if(stable){

				/* 更新信号强度 */
				calc_loss(spsa_params);

				/* 扰动稳定,估计梯度 */
				float tmp_plus  = spsa_params->Soutput;
				float tmp_minus = spsa_params->last_Soutput;

				/* 更新梯度 */
				spsa_params->theta_k_g_k_hat[AXIS_PITCH] = (tmp_plus - tmp_minus)/(spsa_params->ck * spsa_params->delta_k[AXIS_PITCH]);
				if(spsa_params->theta_k_g_k_hat[AXIS_PITCH] < GRADE_PITCH_MIN) spsa_params->theta_k_g_k_hat[AXIS_PITCH] = GRADE_PITCH_MIN;
				if(spsa_params->theta_k_g_k_hat[AXIS_PITCH] > GRADE_PITCH_MAX) spsa_params->theta_k_g_k_hat[AXIS_PITCH] = GRADE_PITCH_MAX;

				spsa_params->theta_k_g_k_hat[AXIS_YAW] 	 = (tmp_plus - tmp_minus)/(spsa_params->ck * spsa_params->delta_k[AXIS_YAW]);
				if(spsa_params->theta_k_g_k_hat[AXIS_YAW] < GRADE_YAW_MIN) spsa_params->theta_k_g_k_hat[AXIS_YAW] = GRADE_YAW_MIN;
				if(spsa_params->theta_k_g_k_hat[AXIS_YAW] > GRADE_YAW_MAX) spsa_params->theta_k_g_k_hat[AXIS_YAW] = GRADE_YAW_MAX;

				spsa_params->last_Soutput = spsa_params->Soutput;

				/* 更新下一个位置 */
				spsa_params->theta_plus[AXIS_PITCH] = spsa_params->theta_k[AXIS_PITCH] + spsa_params->ak * spsa_params->theta_k_g_k_hat[AXIS_PITCH];
				spsa_params->theta_plus[AXIS_YAW] = spsa_params->theta_k[AXIS_YAW] + spsa_params->ak * spsa_params->theta_k_g_k_hat[AXIS_YAW];

				/* 进入等待,等待位置稳定 */
				spsa_params->mode = PUBLISH_DISTRITUTION;
			}
		}
		break;
	}
}







