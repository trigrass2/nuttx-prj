/****************************************************************************
 *  apps/my_project/satellite_attitude_control/pid.c
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
#include <satellite_attitude_control/adrc.h>
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
static void reset_I(struct algorithm_pid_t *pid)
{
    pid->integrator = 0;
	// we use NAN (Not A Number) to indicate that the last
	// derivative value is not valid
    pid->last_derivative = NAN;
    pid->info.I = 0;
}

/****************************************************************************
 * Name: get_pid
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
float get_pid(struct algorithm_pid_t *pid, float error, float scaler)
{
    uint32_t tnow = (uint32_t) (hrt_absolute_time()/1000);
    uint32_t dt = tnow - pid->last_t;
    float output            = 0;
    float delta_time;

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

    // Compute proportional component
    pid->info.P = error * pid->params.kp;
    output += pid->info.P;

    // Compute derivative component if time has elapsed
    if ((fabsf(pid->params.kd) > 0) && (dt > 0)) 
    {
        float derivative;

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

//        if(pid->iir_filter_aFilt != NULL && pid->iir_filter_bFilt != NULL)
//        {
//            derivative = iir_filter(pid->last_derivative,pid->iir_filter_aFilt,pid->iir_filter_bFilt);
//        }
//        else
//        {
//            // discrete low pass filter, cuts out the
//            // high frequency noise that can drive the controller crazy
//            float RC = 1/(2 * M_PI * pid->params.fCut);
//            derivative = pid->last_derivative +( (delta_time / (RC + delta_time)) *(derivative - pid->last_derivative) );
//        }


        // update state
        pid->last_error         = error;
        pid->last_derivative    = derivative;

        // add in derivative component
        pid->info.D = pid->params.kd * derivative;
        output += pid->info.D;
    }

    // scale the P and D components
    output *= scaler;
    pid->info.D *= scaler;
    pid->info.P *= scaler;

    if(pid->axis_id == 0x20 || pid->axis_id == 0x21)//yaw轴控制
    {
        //积分分离
        if(fabsf(error) < 15 && fabsf(error) > 0.3)
        {
          // Compute integral component if time has elapsed
          if ((fabsf(pid->params.ki) > 0) && (dt > 0))
          {
              pid->integrator += (error * pid->params.ki) * scaler * delta_time;
              if (pid->integrator < -pid->params.i_max) {
                pid->integrator = -pid->params.i_max;
              } else if (pid->integrator > pid->params.i_max) {
                pid->integrator = pid->params.i_max;
              }
              pid->info.I = pid->integrator;

              output += pid->integrator;
          }
        }

    }
    else
    {
      //积分分离
      if(fabsf(error) < 10 && fabsf(error) > 0.1)
      {
        // Compute integral component if time has elapsed
        if ((fabsf(pid->params.ki) > 0) && (dt > 0)) 
        {
            pid->integrator += (error * pid->params.ki) * scaler * delta_time;
            if (pid->integrator < -pid->params.i_max) {
              pid->integrator = -pid->params.i_max;
            } else if (pid->integrator > pid->params.i_max) {
              pid->integrator = pid->params.i_max;
            }
            pid->info.I = pid->integrator;

            output += pid->integrator;
        }
      }
    }

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
float td_test_params[2][4]=
{
    /*TD跟踪微分器   改进最速TD,h0=N*h */
    /*r     h      N0  c */
    {10000 ,0.005 , 1, 1},
    {100000 ,0.001 , 4, 1},
};

float get_adrc_pid(struct algorithm_pid_t *pid, float in_need, float out_system, float scaler)
{
    float x1_delta = 0;
    float z1_delta = 0;

    float track_h0 = 0;
    float track_cx2 = 0;

    float filter_h0 = 0;
    float filter_cx2 = 0;  

    struct adrc_td *track_filter = pid->td_params;

    uint32_t tnow;
    uint32_t dt;
    float delta_time; 

    if (pid->last_t == 0) 
    {
      dt = 0;
      // if this PID hasn't been used for a full second then zero
      // the intergator term. This prevents I buildup from a
      // previous fight mode from causing a massive return before
      // the integrator gets a chance to correct itself
      reset_I(pid);

      pid->params.kp = 1.8;
      pid->params.ki = 0.2;
      pid->params.kd = 0.8;

      track_filter->td_r0 = td_test_params[0][0];
      track_filter->td_h0 = td_test_params[0][1];
      track_filter->td_N0 = td_test_params[0][2];
      track_filter->td_damp_c0 = td_test_params[0][3];

      track_filter->td_r1 = td_test_params[1][0];
      track_filter->td_h1 = td_test_params[1][1];
      track_filter->td_N1 = td_test_params[1][2];
      track_filter->td_damp_c1 = td_test_params[1][3];      

      track_filter->td_v1 = in_need;
      track_filter->td_v2 = in_need;

      track_filter->td_z1 = out_system;
      track_filter->td_z2 = out_system;

      printf("get_adrc_pid : all the data is init!!! \n");
    }

    tnow = (uint32_t) (hrt_absolute_time()/1000);
    dt = tnow - pid->last_t;
    pid->last_t = tnow;
    delta_time = (float)dt / 1000.0f;//计算两次函数运行时间间隔
    /*****************************************************************************/
    x1_delta = track_filter->td_v1-in_need;//用x1-v(k)替代x1得到离散更新公式
    track_h0 = track_filter->td_N0*track_filter->td_h0;//用h0替代h，解决最速跟踪微分器速度超调问题
    track_cx2 = track_filter->td_v2*track_filter->td_damp_c0;//用cx2代替x2，可以调节“阻尼因子”，使系统避免高频颤震问题(fhan_Input->damp_c 为可调节‘阻尼因子’)

    track_filter->td_fh0 = Fhan_ADRC(x1_delta,track_cx2,track_filter->td_r0,track_h0);

    track_filter->td_v1 += track_filter->td_h0*track_filter->td_v2;//跟新最速跟踪状态量x1
    track_filter->td_v2 += track_filter->td_h0*track_filter->td_fh0;//跟新最速跟踪状态量微分x2(fh <= r)
    /*****************************************************************************/
    z1_delta = track_filter->td_z1 - out_system;
    filter_h0 = track_filter->td_N1*track_filter->td_h1;//用h0替代h，解决最速跟踪微分器速度超调问题
    filter_cx2 = track_filter->td_z2*track_filter->td_damp_c1;//用cx2代替x2，可以调节“阻尼因子”，使系统避免高频颤震问题(fhan_Input->damp_c 为可调节‘阻尼因子’)  

    track_filter->td_fh1 = Fhan_ADRC(z1_delta,filter_cx2,track_filter->td_r1,filter_h0);

    track_filter->td_z1 += track_filter->td_h1*track_filter->td_z2;//跟新最速跟踪状态量x1
    track_filter->td_z2 += track_filter->td_h1*track_filter->td_fh1;//跟新最速跟踪状态量微分x2(fh <= r)
    /*****************************************************************************/
    track_filter->e1 = track_filter->td_v1 - track_filter->td_z1;//误差项
    track_filter->e2 = track_filter->td_v2 - track_filter->td_z2;//误差微分项
    // Compute integral component if time has elapsed
    if ((fabsf(pid->params.ki) > 0) && (dt > 0)) 
    {
        track_filter->e0 += track_filter->e1 * scaler * delta_time;
        if (pid->integrator < -pid->params.i_max) 
        {
          track_filter->e0 = -pid->params.i_max;
        } 
        else if (pid->integrator > pid->params.i_max) 
        {
          track_filter->e0 = pid->params.i_max;
        }
    }
     /*****************************************************************************/
    // Compute proportional component
    pid->info.P = track_filter->e1 * pid->params.kp;
    pid->info.D = track_filter->e2 * pid->params.kd;
    pid->info.I = track_filter->e0 * pid->params.ki;

    track_filter->u0 = pid->info.P + pid->info.I + pid->info.D;

    // Limit the output value
    if (track_filter->u0 < -pid->params.o_max) {
    	track_filter->u0 = -pid->params.o_max;
    } else if (track_filter->u0 > pid->params.o_max) {
    	track_filter->u0 = pid->params.o_max;
    }


    pid->info.desired = track_filter->u0;

    // Enable or Disable PID output
    if(pid->enable){
    	return track_filter->u0;
    }else{
    	pid->info.desired = 0.0;
    	return 0;
    }
}



