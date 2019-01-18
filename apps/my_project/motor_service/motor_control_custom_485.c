//#ifdef CONFIG_MOTOR_DRIVER_CUSTOM_485
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <nuttx/timers/drv_hrt.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include "pc_control/pc_control.h"
#include <nuttx/input/hall55100.h>
#include <motor_service/elmo_app.h>
#include "uORB/uorb/uORB.h"
#include "uORB/topic/pc_motor_cmd_uorb.h"
#include "uORB/topic/motor_state_uorb.h"
#include <motor_service/motor_fppa.h>
#include <motor_service/motor_serial_msg.h>
#include <motor_service/motor_serial_link.h>

extern FPPA_MOTOR_DEVICE Motor_Devive[DEVICE_MAX_MOTOR_NO];
extern struct motor_state_uorb_s t_motor_state_mesg;
extern orb_advert_t att_motor_state_pub;
extern struct msg_info_s motor_info[3];

int test_rev_msg_from;

void set_drv_posi(int32_t posi1, int32_t posi2, int32_t posi3);
void get_drv_info(uint8_t id);
void link_parse(uint8_t *ch, int length);

float test_dis_cmd[6];

int Deal_Motor_Cmd_Custom_485(FPPA_MOTOR_DEVICE motor_dev_fd[],struct pc_motor_cmd_uorb_s *contrl_cmd)
{
	int ret,err,motor_no;
	static int position[3];
	char mask = 0;
	//static int speed[3];

	err = DO_CMD_ERR_FAULT;
	/*if (motor_dev_fd[motor_no].zero_position >  LIMIT_POSITION_ANGLE_HARDWARE)
	{
		if ((contrl_cmd->cmd[motor_no] != 3) && 
			(contrl_cmd->cmd[motor_no] != 17) &&
			(contrl_cmd->cmd[motor_no] != 14) )//just can serch zreo
		{
			err = DO_CMD_ERR_ILLICIT;
			return err;
		}
	}*/
	if (motor_dev_fd[0].uart_fd <= 0)
	{
		err = DO_CMD_ERR_FILE;
			return err;
	}
	for (motor_no = 0; motor_no < APP_MAX_MOTOR_NO; motor_no++)
	{
		switch (contrl_cmd->cmd[motor_no])
			{
			case 0:
				break;
			case 1://read state
			case 16://stop
			case 2://restart
			case 17://search zero position
			case 3://zero position
				 break;
			case 4://ABSOLUTE position
				//if ((contrl_cmd->params[motor_no].fdata > LIMIT_ELMO_POSITION_ANGLE_SOFTWARE) ||
				//	(contrl_cmd->params[motor_no].fdata < -LIMIT_ELMO_POSITION_ANGLE_SOFTWARE))
				//{

				//	err = DO_CMD_ERR_OUTRANGE;
				//	continue;
				//}
				position[motor_no] = (int)contrl_cmd->params[motor_no].fdata;//////////////////////////////////
				mask = 0x01<<motor_no;
				break;
			case 5://read position
			case 6://set speed
			case 7://read speed
			case 8://set motor acc
			case 9://read acc
			case 10://set dec
			case 11://read dec
			case 12://set relative position
			case 15://read err code
			case 14://read memory position
			break;
			case 13:
			default:
				contrl_cmd->state = MOTOR_CMD_STATE_ERR;
				err = DO_CMD_ERR_CMDNODEFINE;
				//ret = mq_send(g_send_mqfd, &float_int_data.str[0], 4, 42);
				break;
			}
	}
	if (mask)
	{
		set_drv_posi(position[0],position[1],position[2]);
		test_dis_cmd[0] = (int)position[0];
		test_dis_cmd[1] = (int)position[1] ;
		test_dis_cmd[2] = (int)position[2] ;
	}
	return err;
}

int Deal_Motor_Uart_Read(uint8_t *buf,int data_long)
{
	static point;
	int ret,i;
	ret = 0;
	ret += read(Motor_Devive[0].uart_fd, &buf[point], MOTOR_UART_BUF_RX_SIZE);//can't read all//////////////////////////////////////////////////////////
	if ((ret + point) >= 16)
	{
		ret += point;
		point = 0;
		link_parse(buf,ret);
		test_rev_msg_from = 5;
		if (test_rev_msg_from == 10)
		{
			test_rev_msg_from = 1;
			return 1;
		}
		else if (test_rev_msg_from == 11)
		{
			test_rev_msg_from = 2;
			return 2;
		}
		else if (test_rev_msg_from == 12)
		{
			test_rev_msg_from = 3;
			return 3;
		}

	}
	else
	{
		point += ret;
	}
	
	return 0;
}

/*void Get_Motor_Custom_485(uint8_t buf[],int motor_no,struct motor_state_uorb_s *temp_msg,int wait_time)
{
	if (motor_no > APP_MAX_MOTOR_NO)
	{
		return;
	}
	get_drv_info(10 + motor_no);
	wait_time *= 1000;
	usleep(wait_time);
	if(Deal_Motor_Uart_Read(buf,MOTOR_UART_BUF_RX_SIZE) > 0)
	{
		//temp_msg->cmd[motor_no] = MOTOR_BACK_CMD_TIMER_BACK;
		temp_msg->cmd[motor_no] = MOTOR_BACK_CMD_POSITION_OK;
		temp_msg->state[motor_no] = MOTOR_CMD_STATE_DO;
		if (motor_no < 3)
		{
			temp_msg->params[motor_no].fdata = motor_info[motor_no].posi;
		}
		temp_msg->mask |= 0x01<<motor_no;
	}
}*/

void Get_Motor_Custom_485(uint8_t buf[],struct motor_state_uorb_s *temp_msg,int wait_time)
{
	int motor_no;
	motor_no = Deal_Motor_Uart_Read(buf,MOTOR_UART_BUF_RX_SIZE);
	if(motor_no > 0)
	{
		motor_no--;
		//temp_msg->cmd[motor_no] = MOTOR_BACK_CMD_TIMER_BACK;
		temp_msg->cmd[motor_no] = MOTOR_BACK_CMD_POSITION_OK;
		temp_msg->state[motor_no] = MOTOR_CMD_STATE_DO;
		temp_msg->params[motor_no].fdata = motor_info[motor_no].posi;
		temp_msg->mask |= 0x01<<motor_no;
	}
	else if (test_rev_msg_from == 5)
	{
		temp_msg->mask |= 0x80;
	}
}

