/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file objects_common.cpp
 *
 * Common object definitions without a better home.
 */

/**
 * @defgroup topics List of all uORB topics.
 */

#include "uORB/device/drv_orb_dev.h"

#include "uORB/topic/sensor_combined.h"
ORB_DEFINE(sensor_combined, struct sensor_combined_s,0,0);

#include "uORB/topic/beacon_data_uorb.h"
ORB_DEFINE(beacon_data_uorb, struct beacon_data_uorb_s,0,0);

#include "uORB/topic/motor_state_uorb.h"
ORB_DEFINE(motor_state_uorb, struct motor_state_uorb_s,0,0);

#include "uORB/topic/pc_beacon_cmd_uorb.h"
ORB_DEFINE(pc_beacon_cmd_uorb, struct pc_beacon_cmd_uorb_s,0,0);

#include "uORB/topic/pc_motor_cmd_uorb.h"
ORB_DEFINE(pc_motor_cmd_uorb, struct pc_motor_cmd_uorb_s,0,0);

#include "uORB/topic/imu_data_uorb.h"
ORB_DEFINE(imu_data_uorb, struct imu_data_uorb_s, 0, 0);

#include "uORB/topic/adis16488_uorb.h"
ORB_DEFINE(adis16488_uorb, struct adis16488_uorb_s, 0, 0);

#include "uORB/topic/hmc6343_uorb.h"
ORB_DEFINE(hmc6343_uorb, struct hmc6343_uorb_s, 0, 0);

#include "uORB/topic/gps_data_uorb.h"
ORB_DEFINE(gps_data_uorb, struct gps_data_uorb_s,0,0);

#include "uORB/topic/satellite_point_uorb.h"
ORB_DEFINE(satellite_point_uorb, struct satellite_point_uorb_s,0,0);

#include "uORB/topic/pid_turning_data_uorb.h"
ORB_DEFINE(pid_turning_data_uorb, struct pid_turning_data_uorb_s,0,0);

#include "uORB/topic/beam_attitude_uorb.h"
ORB_DEFINE(beam_attitude_uorb, struct beam_attitude_uorb_s,0,0);

#include "uORB/topic/temperature_data_uorb.h"
ORB_DEFINE(temperature_data_uorb, struct temperature_data_uorb_s,0,0);

#include "uORB/topic/pc_PID_cmd_uorb.h"
ORB_DEFINE(pc_PID_cmd_uorb, struct pc_PID_cmd_uorb_s,0,0);

#include "uORB/topic/pc_PID_params_uorb.h"
ORB_DEFINE(pc_PID_params_uorb, struct pc_PID_params_uorb_s,0,0);

#include "uORB/topic/pc_satellite_cmd_uorb.h"
ORB_DEFINE(pc_satellite_cmd_uorb, struct pc_satellite_cmd_uorb_s,0,0);

#include "uORB/topic/pc_gps_cmd_uorb.h"
ORB_DEFINE(pc_gps_cmd_uorb, struct pc_gps_cmd_uorb_s,0,0);

#include "uORB/topic/attitude_set_point_uorb.h"
ORB_DEFINE(attitude_set_point_uorb, struct attitude_set_point_uorb_s,0,0);

#include "uORB/topic/attitude_control_status_uorb.h"
ORB_DEFINE(attitude_control_status_uorb, struct attitude_control_status_uorb_s,0,0);

