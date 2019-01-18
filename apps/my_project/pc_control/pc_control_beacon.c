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


//PC_CMD_STRUCT	Beacon_cmd;

int Deal_PC_Beacon_Data(unsigned char r_str[],
			  int r_data_long,
			  unsigned char s_str[],
			  struct pc_beacon_cmd_uorb_s *send_uorb
			  )
{
	int err;
	Float_Int_type float_int_data;
	err = 0;
	send_uorb->addr = 10;
	send_uorb->cmd = r_str[3];
	send_uorb->state  = PC_CMD_STATE_CREATE;
	if ((r_str[3] & 0x80) == 0)
	{
		float_int_data.str[0] = r_str[4];
		if (r_str[3] == 1)
		{
			float_int_data.str[1] = r_str[5];
			float_int_data.str[2] = r_str[6];
			float_int_data.str[3] = r_str[7];
			send_uorb->params.u_intdata  = float_int_data.u_intdata;
		}
		else
		{
			float_int_data.str[1] = 0;
			float_int_data.str[2] = 0;
			float_int_data.str[3] = 0;
			send_uorb->params.s_intdata = float_int_data.s_intdata;
		}
	
	}
	
	return err;

}

//

int Deal_Beacon_PC_Data(struct beacon_data_uorb_s *read_uorb,
						char s_str[])
{
	static int atuo_back_times;
	int err = 0;
	//Float_Int_type float_int_data;
	if (read_uorb->cmd == BEACON_UORB_CMD_AUTO_DATA)
	{
		atuo_back_times++;
#ifdef CONFIG_BEACON_AUTO_BACK_INTERVAL
		if (atuo_back_times > CONFIG_BEACON_AUTO_BACK_INTERVAL)
		{
			atuo_back_times = 0;
			s_str[1] = 8;
			s_str[2] = 10;
			s_str[3] = BEACON_UORB_CMD_AUTO_DATA;
			float_int_data.fdata = read_uorb->beacon_signal.power;
			s_str[4] = float_int_data.str[0];
			s_str[5] = float_int_data.str[1];
			s_str[6] = float_int_data.str[2];
			s_str[7] = float_int_data.str[3];
			s_str[8] = 0;
			if (read_uorb->beacon_signal.effective)
			{
				s_str[8] |= 0x01;
			}
			if (read_uorb->beacon_signal.lock)
			{
				s_str[8] |= 0x02;
			}
		}
#endif
	}
	else if (read_uorb->cmd == PC_APP_CMD_ERR)
	{
		s_str[1] = 7;
		s_str[2] = 10;
		s_str[3] = read_uorb->cmd;
		s_str[4] = read_uorb->params.str[0];
		s_str[5] = read_uorb->params.str[1];
		s_str[6] = read_uorb->params.str[2];
		s_str[7] = 0;
	}
	else
	{
		s_str[1] = 7;
		s_str[2] = 10;
		s_str[3] = read_uorb->cmd;
		s_str[4] = read_uorb->params.str[0];
		s_str[5] = read_uorb->params.str[1];
		s_str[6] = read_uorb->params.str[2];
		s_str[7] = read_uorb->params.str[3];
	}
	return err;
}
//==================================================
//Deal_GPS_Beacon_Data
//sate_fre:input fre MHz
//return_type:GPS_BEACON_RETURN_FRE,GPS_BEACON_RETURN_VOL
//==================================================
int Deal_GPS_Beacon_Data(float sate_fre,struct pc_beacon_cmd_uorb_s *send_uorb, char return_type)
{
	uint32_t a;
	uint32_t temp;
	
	sate_fre *= 1000;//to kHz,
	temp = (uint32_t)sate_fre;
	if((temp > SATE_MAX_FRE) || (temp < SATE_MIN_FRE))
	{
		syslog(LOG_WARNING, "pc_control_beacon: input fre out of range %d \n",sate_fre);
		return -1;
	}
	if(return_type == GPS_BEACON_RETURN_FRE)
	{
		a = BEACON_MIN_FRE;
		a /= 1000;
		if(temp >= BEACON_SWITCH)
		{
			a = BEACON_SWITCH - a;
			temp -= a;
		}
		else
		{
			a = SATE_MIN_FRE - a;
			temp -= a;
		}
		temp *= 1000;//to Hz
		send_uorb->cmd = BEACON_UORB_CMD_SET_FRE;
		send_uorb->params.u_intdata = temp;
		send_uorb->addr = PC_ADDR_BEACON;
		send_uorb->state = MOTOR_CMD_STATE_CREATE;
	}
	else if(return_type == GPS_BEACON_RETURN_VOL)
	{
		if(temp >= BEACON_SWITCH)
		{
			temp = BEACON_UORB_LNB_POWER_18_2;
		}
		else
		{
			temp = BEACON_UORB_LNB_POWER_13_4;
		}
		send_uorb->cmd = BEACON_UORB_CMD_SET_LNB_POWER;
		send_uorb->params.u_intdata  = temp;
		send_uorb->addr = PC_ADDR_BEACON;
		send_uorb->state = MOTOR_CMD_STATE_CREATE;
	}
	return 0;
}



