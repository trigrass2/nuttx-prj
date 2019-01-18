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

struct gps_time
{
	int32_t microseconds;
	char year;
	char month;
	char day;
	char hour;
	char minutes;
	char second;
	char week;
	char reserve;
};

#define GPS_TYPE_GPS	1
#define GPS_TYPE_BD		2
#define GPS_TYPE_GN		3
#define GPS_TYPE_NONE	0

#define GPS_TYPE_SET	100

#define GPS_STATE_NONE	0
#define GPS_STATE_SET	100

#ifdef __cplusplus
struct __attribute__ ((visibility ("default"))) gps_data_uorb_s {
#else
struct gps_data_uorb_s {
#endif
	float longitude;//jingdu
	float latitude;//weidu
	float course;//hang xiang
	float altitude;//hai ba
	float speed;

	float azimuth;//fang wei
	

	float hdop;//shui ping
	float pdop;//zong he
	float vdop;//chui zhi

	struct gps_time time;
	char satellite_no;
	char gps_type;
	char gps_state;
	char reserve;
	
#ifdef __cplusplus
    static constexpr int32_t RELATIVE_TIMESTAMP_INVALID = 2147483647;
 
#endif
};

ORB_DECLARE(gps_data_uorb);



