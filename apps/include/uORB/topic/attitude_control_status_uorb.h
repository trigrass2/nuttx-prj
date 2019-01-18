/****************************************************************************
 *
 *   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 
#pragma once
 
#include <stdint.h>
#ifdef __cplusplus
#include <cstring>
#else
#include <string.h>
#endif
 
#include "../uorb/uORB.h"
 
#ifndef __cplusplus
#define RELATIVE_TIMESTAMP_INVALID 2147483647
 
#endif


#ifdef __cplusplus
struct __attribute__ ((visibility ("default"))) attitude_control_status_uorb_s {
#else
struct attitude_control_status_uorb_s {
#endif
	int mode;					/* 当前运行模式 */
	float curr_roll;			/* 当前实时横滚轴 */
	float curr_pitch;			/* 当前实时俯仰轴 */
	float curr_yaw;				/* 当前实时航向轴 */
	bool  stable;				/* 标示当前姿态是否控制稳定 */
#ifdef __cplusplus
    static constexpr int32_t RELATIVE_TIMESTAMP_INVALID = 2147483647;
 
#endif
};
 

ORB_DECLARE(attitude_control_status_uorb);

