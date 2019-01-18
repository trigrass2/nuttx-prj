#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mqueue.h>
#include "uORB/topic/pc_PID_cmd_uorb.h"
#include "pc_control/pc_control.h"



int Deal_PC_PID_Data(unsigned char r_str[],
					   int r_data_long,
					   unsigned char s_str[],
					   struct pc_PID_cmd_uorb_s *send_uorb)
{
	//int i;
	if (r_data_long < (DATA_LONG_PC_PID + 6))
	{
		syslog(LOG_ERR, "pc_control_PID: read pc data long err！:\n",r_data_long);
		return 1;
	}
	memcpy(send_uorb,&r_str[4],DATA_LONG_PC_PID);
	/*send_uorb->id 			= s_str[4];
	send_uorb->op_mode 		= s_str[5];
	send_uorb->enable  		= s_str[6];
	send_uorb->reserve_byte = s_str[7];
	send_uorb->Kp 			= Char_Fint(&r_str[8],4).fdata;
	send_uorb->Ki 			= Char_Fint(&r_str[12],4).fdata;
	send_uorb->Kd 			= Char_Fint(&r_str[16],4).fdata;
	send_uorb->Kf 			= Char_Fint(&r_str[20],4).fdata;
	send_uorb->iMax 		= Char_Fint(&r_str[24],4).fdata;
	send_uorb->iMin 		= Char_Fint(&r_str[28],4).fdata;
	send_uorb->fCut 		= Char_Fint(&r_str[32],4).fdata;
	send_uorb->target_value = Char_Fint(&r_str[36],4).fdata;
	send_uorb->reserve1  	= Char_Fint(&r_str[40],4).fdata;
	send_uorb->reserve2 	= Char_Fint(&r_str[44],4).fdata;
	send_uorb->reserve3 	= Char_Fint(&r_str[48],4).fdata;
	send_uorb->reserve3 	= Char_Fint(&r_str[52],4).fdata;
	*/
	return 0;

}
