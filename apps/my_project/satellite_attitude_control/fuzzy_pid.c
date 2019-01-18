/****************************************************************************
 *  apps/my_project/satellite_attitude_control/fuzzy_pid.c
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

#include <satellite_attitude_control/fuzzy_pid.h>
#include <nuttx/timers/drv_hrt.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: reset_I
 *
 * Description:
 *   clean the integrator.
 *
 * Input Parameters:
 *   pid    - pid-specific data
 *
 ****************************************************************************/
static void reset_I(struct algorithm_fuzzy_pid_t *pid)
{
    pid->integrator = 0;
	// we use NAN (Not A Number) to indicate that the last
	// derivative value is not valid
    pid->last_derivative = NAN;
    pid->info.I = 0;
}

/****************************************************************************
 * Name: fuzzy_out
 *
 * Description:
 *  the fuzzy parameters .
 *
 * Input Parameters:
 *   pid    - pid-specific data
 *
 ****************************************************************************/
void fuzzy_out(struct algorithm_fuzzy_pid_t *pid,float error,float delta_error)
{
   unsigned char  i,j;
   float psum=0.0,isum=0.0,dsum=0.0;
   float l_BP[10]={0,0,0,0,0,0,0,0,0,0};
   float l_BD[10]={0,0,0,0,0,0,0,0,0,0};

   /* 确定偏差位置 */
   for(i=0; i<pid->tab.row; i++){

     if(error <= pid->tab.Edot[i]) break;

   }

   /*
    * 偏差位置分为两种情况(线性化处理偏差)
    * 1. 处于两头
    * 2. 处于中间
    */
   if(i==0)l_BP[0]=100;
   else if(i == pid->tab.row ) l_BP[pid->tab.row-1] = 100;
   else if(i > 0 && i<pid->tab.row){

     l_BP[i] 	= (float)100.0*(error-pid->tab.Edot[i-1])/(pid->tab.Edot[i]-pid->tab.Edot[i-1]);
     l_BP[i-1]	= (float)100.0*(pid->tab.Edot[i]-error)/(pid->tab.Edot[i]-pid->tab.Edot[i-1]);

   }

   /* 确定微分位置 */
   for(j=0;j<pid->tab.col;j++){

     if(delta_error<=pid->tab.ECdot[j])break;

   }

   /*
    * 微分位置分为两种情况(线性化处理偏差)
    * 1. 处于两头
    * 2. 处于中间
    */
   if(j==0)l_BD[0]=100;
   else if(j==pid->tab.col) l_BD[pid->tab.col-1]=100;
   else if(j>0 && j<pid->tab.col){

     l_BD[j]   = (float)100.0*(delta_error-pid->tab.ECdot[j-1])/(pid->tab.ECdot[j]-pid->tab.ECdot[j-1]);
     l_BD[j-1] = (float)100.0*(pid->tab.ECdot[j]-delta_error)/(pid->tab.ECdot[j]-pid->tab.ECdot[j-1]);

   }

   /*
    * 合成输出
    */
   for(i=0; i<pid->tab.row; i++){

     for(j=0; j<pid->tab.col; j++){

       psum+=(float)l_BP[i] * l_BD[j] * (pid->tab.ptab[i][j]);
       isum+=(float)l_BP[i] * l_BD[j] * (pid->tab.itab[i][j]);
       dsum+=(float)l_BP[i] * l_BD[j] * (pid->tab.dtab[i][j]);

     }
   }

//	//bang-bang控制.
//	if(fabsf(error) > 0.8){
//		 pid->tab.kp = pid->params.kp * pid->params.kf;
//	}else{
//		pid->tab.kp = pid->params.kp * (psum/1000000.0);
//	}

   pid->tab.kp = pid->params.kp * (psum/1000000.0);
//	   pid->tab.ki = pid->params.ki * (isum/1000000.0);
   pid->tab.ki = pid->params.ki;
   pid->tab.kd = pid->params.kd * (dsum/1000000.0);
}


/****************************************************************************
 * Name: get_fuzzy_pid
 *
 * Description:
 *   Generic PID algorithm, with EEPROM-backed storage of constants.
 *
 * Input Parameters:
 *   pid    - pid-specific data
 *   error  - error of the target and feedback
 *   scaler - scale the P 、I and D components
 *
 * Returned Value:
 *   the result of PID algorithm.
 *
 ****************************************************************************/
float get_fuzzy_pid(struct algorithm_fuzzy_pid_t *pid, float error, float scaler)
{
    uint32_t tnow = (uint32_t) (hrt_absolute_time()/1000);
    uint32_t dt = tnow - pid->last_t;
    float output            = 0;
    float delta_time;
    float derivative;

    if (pid->last_t == 0 || dt > 1000)
    {
      dt = 0;
      // if this PID hasn't been used for a full second then zero
      // the intergator term. This prevents I buildup from a
      // previous fight mode from causing a massive return before
      // the integrator gets a chance to correct itself
      reset_I(pid);
    }

    pid->last_t = tnow;

    delta_time = (float)dt / 1000.0f;

    // Compute derivative component if time has elapsed
    if ((fabsf(pid->params.kd) > 0) && (dt > 0))
    {
        if (isnan(pid->last_derivative))
        {
          // we've just done a reset, suppress the first derivative
          // term as we don't want a sudden change in input to cause
          // a large D output change
          derivative = 0;
          pid->last_derivative = 0;

        } else {

          derivative = (error - pid->last_error) / delta_time;

        }

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        float RC = 1/(2 * M_PI * pid->params.fCut);
        derivative = pid->last_derivative +( (delta_time / (RC + delta_time)) *(derivative - pid->last_derivative) );

        // update state
        pid->last_error         = error;
        pid->last_derivative    = derivative;
    }

    // 执行模糊逻辑
    fuzzy_out(pid,pid->last_error,pid->last_derivative);

    // Compute proportional component
    pid->info.P = error * pid->tab.kp;
    output += pid->info.P;

    // add in derivative component
    pid->info.D = pid->tab.kd * derivative;
    output += pid->info.D;

    // scale the P and D components
    output *= scaler;
    pid->info.D *= scaler;
    pid->info.P *= scaler;


	// Compute integral component if time has elapsed
	if ((fabsf(pid->tab.ki) > 0) && (dt > 0))
	{
		//积分分离
		if(fabsf(error) < pid->params.i_threshold){

			pid->integrator += (error * pid->tab.ki) * scaler * delta_time;

			if (pid->integrator < -pid->params.i_max) {

			  pid->integrator = -pid->params.i_max;

			} else if (pid->integrator > pid->params.i_max) {

			  pid->integrator = pid->params.i_max;

			}

			pid->info.I = pid->integrator;

			output += pid->integrator;
		}
	}

//	output += pid->params.kf * error;

    // Limit the output value
    if (output < -pid->params.o_max) {

    	output = -pid->params.o_max;

    } else if (output > pid->params.o_max) {

    	output = pid->params.o_max;

    }

    pid->info.desired = output;

    // Enable or Disable PID output
    if(pid->enable){

    	return output;

    }else{

    	pid->info.desired = 0.0;

    	return 0;
    }
}

