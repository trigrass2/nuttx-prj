#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <mqueue.h>
#include <nuttx/power/motor.h>
#include "beacon/beacon.h"
#include "pc_control/pc_control.h"
#include "uORB/topic/beacon_data_uorb.h"
#include "uORB/topic/pc_beacon_cmd_uorb.h"
#include "uORB/topic/pc_satellite_cmd_uorb.h"
#include "uORB/topic/pc_gps_cmd_uorb.h"
#include "uORB/topic/gps_data_uorb.h"

#define PC_SATE_CMD_SET_SATE	0XC2
#define PC_SATE_CMD_SET_GPS		0XC1
#define PC_SATE_CMD_READ_GPS	0XC0
#define PC_SATE_CMD_SET_POINT	0XC3

extern struct gps_data_uorb_s 			R_GPS_Msg;

int Deal_GPS_PC_param(unsigned char s_str[],struct gps_data_uorb_s *r_gps_msg)
{
	Float_Int_type float_int_data;
	s_str[1] = 0X31;
	s_str[2] = PC_ADDR_GPS;
	s_str[3] = 1;
	
	s_str[4] = r_gps_msg->gps_state;

	float_int_data.fdata = r_gps_msg->longitude;
	memcpy(&s_str[5],&float_int_data.str[0],4);

	float_int_data.fdata = r_gps_msg->latitude;
	memcpy(&s_str[9],&float_int_data.str[0],4);

	float_int_data.fdata = r_gps_msg->course;
	memcpy(&s_str[13],&float_int_data.str[0],4);

	float_int_data.fdata = r_gps_msg->altitude;
	memcpy(&s_str[17],&float_int_data.str[0],4);

	float_int_data.fdata = r_gps_msg->speed;
	memcpy(&s_str[21],&float_int_data.str[0],4);

	float_int_data.fdata = r_gps_msg->azimuth;
	memcpy(&s_str[25],&float_int_data.str[0],4);

	float_int_data.fdata = r_gps_msg->pdop;
	memcpy(&s_str[29],&float_int_data.str[0],4);

	float_int_data.fdata = r_gps_msg->hdop;
	memcpy(&s_str[33],&float_int_data.str[0],4);

	float_int_data.fdata = r_gps_msg->vdop;
	memcpy(&s_str[37],&float_int_data.str[0],4);

	s_str[41] = r_gps_msg->satellite_no;
	s_str[42] = r_gps_msg->time.year;
	s_str[43] = r_gps_msg->time.month;
	s_str[44] = r_gps_msg->time.day;
	s_str[45] = r_gps_msg->time.hour;
	s_str[46] = r_gps_msg->time.minutes;
	s_str[47] = r_gps_msg->time.second;
	s_str[48] = r_gps_msg->time.microseconds;
	s_str[49] = r_gps_msg->time.microseconds >> 8;
	return 0;
}

int Deal_PC_Satellite_GPS_Cmd(unsigned char r_str[],
					   int r_data_long,
					   unsigned char s_str[],
					   struct pc_satellite_cmd_uorb_s *send_uorb_sate,
					   struct pc_gps_cmd_uorb_s *send_uorb_gps)
{
	//int i;
	
	if (r_str[3] == PC_SATE_CMD_SET_SATE)
	{
		if (r_data_long < (sizeof(struct pc_satellite_cmd_uorb_s) + 6))
		{
			syslog(LOG_ERR, "pc_control_gsp_sate: read pc sate data long err！:\n",r_data_long);
			return 0xff;
		}
		memcpy(send_uorb_sate,&r_str[4],sizeof(struct pc_satellite_cmd_uorb_s));
		s_str[1] = 7;
		return 1;
	}
	else if(r_str[3] == PC_SATE_CMD_SET_POINT)
	{
		if (r_data_long < (sizeof(struct pc_satellite_cmd_uorb_s) + 6))
		{
			syslog(LOG_ERR, "pc_control_gsp_sate: read pc point data long err！:\n",r_data_long);
			return 0xff;
		}
		memcpy(send_uorb_sate,&r_str[4],sizeof(struct pc_satellite_cmd_uorb_s));
		s_str[1] = 7;
		return 3;
	}
	else if (r_str[3] == PC_SATE_CMD_SET_GPS)
	{
		if (r_data_long < (DATA_LONG_PC_GPS + 6))
		{
			syslog(LOG_ERR, "pc_control_gsp_sate: read pc sate data long err！:\n",r_data_long);
			return 0xff;
		}
		memcpy(send_uorb_gps,&r_str[4],DATA_LONG_PC_GPS);
		return 2;
	}
	else if (r_str[3] == PC_SATE_CMD_READ_GPS)
	{
		Deal_GPS_PC_param(s_str,&R_GPS_Msg);
		s_str[2] = PC_ADDR_SATELLLITE;
		s_str[3] = PC_SATE_CMD_READ_GPS;
	}
	return 0;
}

