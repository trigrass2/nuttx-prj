#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <mqueue.h>
#include <poll.h>
#include <nuttx/timers/drv_hrt.h>

#include <sys/ioctl.h>
#include <nuttx/fs/ioctl.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>

#include <nuttx/pthread.h>
#include <nuttx/power/motor.h>

#include "pc_control/pc_control.h"
#include "canopen_stack/can_device.h"
#include "canopen_stack/canfestival.h"
#include <nuttx/input/hall55100.h>
#include <motor_service/elmo_app.h>

#include "uORB/uorb/uORB.h"
#include "uORB/topic/pc_motor_cmd_uorb.h"
#include "uORB/topic/motor_state_uorb.h"
#include <motor_service/motor_fppa.h>

#include <math.h>

#include <nuttx/power/elmo_db4x_pwm_io.h>
#define CHANNEL(ch) (ch)
int test_motor_flag;
int watchdog_motor_state;

extern FPPA_MOTOR_DEVICE Motor_Devive[DEVICE_MAX_MOTOR_NO];
extern struct motor_state_uorb_s t_motor_state_mesg;
extern orb_advert_t att_motor_state_pub;
extern struct Motor_Ttask_Struct Motor_Ttask_s;

volatile unsigned char Hall_Irq_Count[6]; //

float test_dis_posi[6];

static uint8_t	uart_rx_buf[MOTOR_UART_BUF_RX_SIZE];//////////////////////////////////////////////////////////////////////////////////

int test_mav_time_ms;
int test_mav_value;
int test_mav_control_t;


int Deal_Motor_Cmd_Elmo(struct motor_params_s motor_s[],struct motor_state_s state_s[],
				   FPPA_MOTOR_DEVICE motor_dev_fd[],struct pc_motor_cmd_uorb_s *contrl_cmd,
				   struct motor_pwm_io_param_s temp_pwm[],unsigned char motor_no);
int Deal_Motor_Cmd_Custom_485(FPPA_MOTOR_DEVICE motor_dev_fd[],struct pc_motor_cmd_uorb_s *contrl_cmd);
int Deal_Motor_pwm_Cmd_Elmo(struct motor_pwm_io_param_s temp_pwm[],FPPA_MOTOR_DEVICE motor_dev_fd[],struct pc_motor_cmd_uorb_s *contrl_cmd,int motor_no);
void Get_Motor_Custom_485(uint8_t buf[],struct motor_state_uorb_s *temp_msg,int wait_time);

int Hall_1_irq_haderler(int irq, FAR void *context, FAR void *arg)
{
	Hall_Irq_Count[0]++;
	return 0;
}
int Hall_2_irq_haderler(int irq, FAR void *context, FAR void *arg)
{
	Hall_Irq_Count[1]++;
	return 0;
}
int Hall_3_irq_haderler(int irq, FAR void *context, FAR void *arg)
{
	Hall_Irq_Count[2]++;
	return 0;
}
int Hall_4_irq_haderler(int irq, FAR void *context, FAR void *arg)
{
	Hall_Irq_Count[3]++;
	return 0;
}
int Hall_5_irq_haderler(int irq, FAR void *context, FAR void *arg)
{
	Hall_Irq_Count[4]++;
	return 0;
}
int Hall_6_irq_haderler(int irq, FAR void *context, FAR void *arg)
{
	Hall_Irq_Count[5]++;
	return 0;
}
//=============================================
//Search_Zero
//motor_s:
//state_s:
//motor_no:
//=============================================

const float motor_offset_str[] = 
{
	0.0,	//READ L1
	0.108,	//READ L2
	0.0,	//READ L3
	0.0,	//SENT L1
	0.0,	//SENT L2
	0.0		//SENT L3
};

int Search_Zero_Speed(struct motor_params_s motor_s[],
				struct motor_state_s state_s[],
				FPPA_MOTOR_DEVICE motor_fds[],
				struct motor_pwm_io_param_s temp_pwm[],
				unsigned char motor_no)
{
	int ret,i;
	int err = 0;
	//float zero_position[];


	int hall_fd[DEVICE_MAX_MOTOR_NO];
	unsigned char temp_state[DEVICE_MAX_MOTOR_NO],flag;
	uint32_t temp_count[DEVICE_MAX_MOTOR_NO];

	int hall_state_temp[DEVICE_MAX_MOTOR_NO];

	float temp_f;

	unsigned char timer_no;
 	unsigned char pwm_chanal;
	float Hall_Irq_Position[DEVICE_MAX_MOTOR_NO][2];//

	//const unsigned char hall_char[] = "/dev/hall1";
	//const float move_position_str[] = {45.0,-5.0,0.9,-0.09,0.009,-0.009,0.0009};
	const float move_speed_str[] = {30.0, -5.0, 0.5, 0, 0};

	if ((motor_no > APP_MAX_MOTOR_NO) || (motor_no < 0))
	{
		return 0xff;
	}

	hall_fd[0] = open("/dev/hall1", O_RDWR);
	if(hall_fd[0] < 0){
	  syslog(LOG_ERR,"motor_control:ERROR:open /dev/hall1 failed :%d\n",hall_fd[0]);
	}
	else
	{
		ret = ioctl(hall_fd[0],HALLIOC_SETCALLBACK,(unsigned long)Hall_1_irq_haderler);
	}
	if (motor_no > 1)
	{
		hall_fd[1] = open("/dev/hall2", O_RDWR);
		if(hall_fd[1] < 0){
		  syslog(LOG_ERR,"motor_control:ERROR:open /dev/hall2 failed :%d\n",hall_fd[1]);
		}
		else
		{
			ret = ioctl(hall_fd[1],HALLIOC_SETCALLBACK,(unsigned long)Hall_2_irq_haderler);
		}
	}
	if (motor_no > 2)
	{
		hall_fd[2] = open("/dev/hall3", O_RDWR);
		if(hall_fd[2] < 0){
		  syslog(LOG_ERR,"motor_control:ERROR:open /dev/hall3 failed :%d\n",hall_fd[2]);
		}
		else
		{
			ret = ioctl(hall_fd[2],HALLIOC_SETCALLBACK,(unsigned long)Hall_3_irq_haderler);
		}
	}
	if (motor_no > 3)
	{
		hall_fd[3] = open("/dev/hall4", O_RDWR);
		if(hall_fd[3] < 0){
		  syslog(LOG_ERR,"motor_control:ERROR:open /dev/hall4 failed :%d\n",hall_fd[3]);
		}
		else
		{
			ret = ioctl(hall_fd[3],HALLIOC_SETCALLBACK,(unsigned long)Hall_4_irq_haderler);
		}
	}
	if (motor_no > 4)
	{
		hall_fd[4] = open("/dev/hall5", O_RDWR);
		if(hall_fd[4] < 0){
		  syslog(LOG_ERR,"motor_control:ERROR:open /dev/hall5 failed :%d\n",hall_fd[4]);
		}
		else
		{
			ret = ioctl(hall_fd[4],HALLIOC_SETCALLBACK,(unsigned long)Hall_5_irq_haderler);
		}
	}
	if (motor_no > 5)
	{
		hall_fd[5] = open("/dev/hall6", O_RDWR);
		if(hall_fd[5] < 0){
		  syslog(LOG_ERR,"motor_control:ERROR:open /dev/hall6 failed :%d\n",hall_fd[5]);
		}
		else
		{
			ret = ioctl(hall_fd[5],HALLIOC_SETCALLBACK,(unsigned long)Hall_6_irq_haderler);
		}
	}

	for (i=0;i<motor_no;i++)
	{
		if (motor_fds[i].control_type != MOTOR_OPMODE_SPEED)
		{
			motor_fds[i].control_type = MOTOR_OPMODE_SPEED;
			ret = ioctl(motor_fds[i].canopen_fd,PWRIOC_SET_MODE, MOTOR_OPMODE_SPEED);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			ret = ioctl(motor_fds[i].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[i]);
			ret = ioctl(motor_fds[i].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[i]);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			if (err)
			{
				syslog(LOG_ERR,"motor_control:ERROR:set motor relative position mode failed %d\n",i);
			}
		}
	}
	syslog(LOG_INFO,"motor_control:Init motor zero point.....\n");
	for (i=0;i<motor_no;i++)
	{
		temp_state[i] = 0;
		temp_count[i] = 0;
		motor_fds[i].zero_position = 0;
		Hall_Irq_Count[i] = 0;
		hall_state_temp[i] = 0;

		if (i > 2)
		{
			timer_no = 1;
			pwm_chanal = i - 3;
		}
		else
		{
			timer_no = 0;
			pwm_chanal = i;
		}

		temp_f = (move_speed_str[0] * PWM_PERCENT_MAX) / 100.0;
		temp_pwm[timer_no].duties[pwm_chanal] = (int)temp_f;
		ret = ioctl(motor_fds[i].pwm_fd, PWRIOC_RUN_SPEED, (unsigned long)((uintptr_t)&temp_pwm[timer_no]));
		state_s[i].state  = MOTOR_STATE_IDLE;
		ret = ioctl(motor_fds[i].canopen_fd, PWRIOC_SET_STATE, (unsigned long)&state_s[i]);
		
		if (ret < 0)
		{
			err = DO_CMD_ERR_FILE;
			syslog(LOG_INFO,"motor_control:serch zero err,pwm control fail :%d\n",ret);
		}
	}
	flag = 0;
	while (1)
	{
		for (i=0;i<motor_no;i++)
		{
			if (Hall_Irq_Count[i] >= 2)
			{
				if (hall_state_temp[i] != 2)
				{
					hall_state_temp[i] = 2;
					if (temp_state[i] == 2)
					{
						ret = ioctl(motor_fds[i].canopen_fd, PWRIOC_GET_STATE, (unsigned long)&state_s[i]);
						if (state_s[i].state == MOTOR_STATE_RUN)//move ok
						{
							Hall_Irq_Position[i][1] = state_s[i].fb.position;
							motor_fds[i].zero_position = (Hall_Irq_Position[i][1] + Hall_Irq_Position[i][0])/2 + motor_offset_str[i];
							flag |= 1<<i;
						}
						else
						{
							Hall_Irq_Count[i] = 0;
							temp_state[i]--;
						}
					}
					else
					{
						Hall_Irq_Count[i] = 0;
					}
					temp_state[i]++;
					if (i > 2)
					{
						timer_no = 1;
						pwm_chanal = i - 3;
					}
					else
					{
						timer_no = 0;
						pwm_chanal = i;
					}
					temp_f = (move_speed_str[temp_state[i]] * PWM_PERCENT_MAX) / 100.0;
					temp_pwm[timer_no].duties[pwm_chanal] = (int)temp_f;
					ret = ioctl(motor_fds[i].pwm_fd, PWRIOC_RUN_SPEED, (unsigned long)((uintptr_t)&temp_pwm[timer_no]));
					temp_count[i] = 0;
					
				}
			}
			else if (Hall_Irq_Count[i] == 1)
			{
				if (hall_state_temp[i] != 1)
				{
					hall_state_temp[i] = 1;
					if (temp_state[i] == 2)
					{
						ret = ioctl(motor_fds[i].canopen_fd, PWRIOC_GET_STATE, (unsigned long)&state_s[i]);
						if (state_s[i].state == MOTOR_STATE_RUN)//
						{
							Hall_Irq_Position[i][0] = state_s[i].fb.position;
						}
						else
						{
							Hall_Irq_Count[i] = 0;
							hall_state_temp[i] = 0;
						}
					}
					temp_count[i] = 0;
				}
				else
				{
					if (temp_count[i] > 500)
					{
						if (Hall_Irq_Count[i] != 2)
						{
							Hall_Irq_Count[i] = 0;
						}
					}
				}
			}
			temp_count[i]++;
			if (temp_count[i] > 10000)
			{
				syslog(LOG_INFO,"motor_control: L%d zero search out of time,state is %d!\n",(i+1),temp_state[i]);
				err = DO_CMD_ERR_FAULT;
				watchdog_motor_state = 1000;
			}
		}
		if ((flag == (0XFF >> (8 - motor_no))) || (err == DO_CMD_ERR_FAULT)) //all motor is complete
		{
			for (i=0;i<motor_no;i++)
			{
				temp_pwm[timer_no].duties[pwm_chanal] = 0;
				ret = ioctl(motor_fds[i].pwm_fd, PWRIOC_RUN_SPEED, (unsigned long)((uintptr_t)&temp_pwm[timer_no]));
			}
			if ((flag == (0XFF >> (8 - motor_no))))
			{
				syslog(LOG_INFO,"motor_control: zero search OK !\n");
				watchdog_motor_state = 0;
			}
			else
			{
				syslog(LOG_INFO,"motor_control: zero search fail,mask is : %d\n",flag);
				watchdog_motor_state = 1000;
			}

			break;
		}
		usleep(1000);

	}

	for (i=0;i<motor_no;i++)
	{
		if (hall_fd[i] > 0)
		{
			close(hall_fd[i]);
		}
	}

	 return err;

}
/****************************************************************************
 * Name: canopen_tx_pollthread
 *
 * Description:
 *   This is the canopen send thread.
 *
 ****************************************************************************/

//void *canopen_tx_pollthread(void *pvarg)
void canopen_tx_fun(void *pvarg)
{
  int ret;

  //int cnt = 0;
  struct motor_params_s params[DEVICE_MAX_MOTOR_NO];
  struct motor_state_s state[DEVICE_MAX_MOTOR_NO];
  
  struct motor_pwm_io_param_s motor_pwm_param[2];

  int poll_ret,i,motor_no;
  bool update = false;
  int error_counter = 0;
  int control_countr = 0;
  watchdog_motor_state = 0;

  double test_f;
  hrt_abstime time_now;
  //int time_cost;


	struct pc_motor_cmd_uorb_s r_pc_mesg;
	memset(&r_pc_mesg, 0, sizeof(r_pc_mesg));
	/* subscribe to sensor_combined topic */
	int pc_mesg_sub_fd = orb_subscribe(ORB_ID(pc_motor_cmd_uorb));
	

	struct pollfd fds[] =
	{
		{ .fd = pc_mesg_sub_fd,   .events = POLLIN },
		//{ .fd = r_motor_back_fd,   .events = POLLIN },
	};
	
	t_motor_state_mesg.mask = 0;
	/* obtained data for the first file descriptor */

	for (i = 0; i < APP_MAX_MOTOR_NO; i++)
	{
		params[i].lock = false;
		params[i].position = 0;
		params[i].acceleration = 720.0;
		params[i].deceleartion = 720.0;
		params[i].speed = 360.0;
	}

	for (i = 0; i < DEVICE_MAX_MOTOR_NO; i++)
	{
		Motor_Devive[i].zero_position = MOTOR_DEFAULT_ZERO;
		Motor_Devive[i].control_type = MOTOR_OPMODE_SPEED;
		//zero_position[i] = 0;
	}
	for (i=0;i<CONFIG_PWM_NCHANNELS;i++)
	{
		motor_pwm_param[0].channels[i] = CHANNEL(i+1);
		motor_pwm_param[0].duties[i] = 10;
		
		motor_pwm_param[1].channels[i] = CHANNEL(i+1);
		motor_pwm_param[1].duties[i] = 10;
	}
	motor_pwm_param[0].frequency = 10000;
	motor_pwm_param[0].initialized = true;
	motor_pwm_param[1].frequency = 10000;
	motor_pwm_param[1].initialized = true;


	test_mav_time_ms = 1000;
	test_mav_value = 100;
	test_mav_control_t = 5;

	while (Motor_Ttask_s.task_state != MOTOR_TASK_STATE_EXIT)
	{


#ifdef CONFIG_MOTOR_DRIVER_CUSTOM_485
		/*for (motor_no = 0; motor_no < APP_MAX_MOTOR_NO; motor_no++)
		{
			Get_Motor_Custom_485(uart_rx_buf, motor_no,&t_motor_state_mesg);
			orb_check(pc_mesg_sub_fd,&update);
			if(update)
			{
				orb_copy(ORB_ID(pc_motor_cmd_uorb), pc_mesg_sub_fd, &r_pc_mesg);
				Deal_Motor_Cmd_Custom_485(&Motor_Devive[0],&r_pc_mesg);
			}
		}
		//if (t_motor_state_mesg.mask == (0XFF >> (8-APP_MAX_MOTOR_NO)))
		if (t_motor_state_mesg.mask == (0XFF >> (8-APP_MAX_MOTOR_NO)))
		{
			orb_publish(ORB_ID(motor_state_uorb), att_motor_state_pub, &t_motor_state_mesg);
			test_dis_posi[0] = t_motor_state_mesg.params[0].fdata;
			test_dis_posi[1] = t_motor_state_mesg.params[1].fdata;
			test_dis_posi[2] = t_motor_state_mesg.params[2].fdata;

		}
		else
		{
			syslog(LOG_ERR, "motor_contrl: read bad:%d\n",t_motor_state_mesg.mask);
		}*/

		while (1)
		{
			Get_Motor_Custom_485(uart_rx_buf, &t_motor_state_mesg,test_mav_control_t);
			if (t_motor_state_mesg.mask & 0x04)
			{
				test_dis_posi[0] = t_motor_state_mesg.params[0].fdata;
				test_dis_posi[1] = t_motor_state_mesg.params[1].fdata;
				test_dis_posi[2] = t_motor_state_mesg.params[2].fdata;
				t_motor_state_mesg.mask = 0;
				break;
			}
			else if (t_motor_state_mesg.mask & 0x80)
			{
				t_motor_state_mesg.mask = 0x40;
			}
			else
			{
				//syslog(LOG_ERR, "motor_contrl: read bad:%d\n",t_motor_state_mesg.mask);
			}
		}
		//============================================================
		//test fun
		control_countr++;
		if (control_countr >= (test_mav_control_t >> 2))
		{
			control_countr = 0;

			time_now = hrt_absolute_time();
			test_f = (double)time_now;
			test_f /= 1000;
			test_f = fmod(test_f,test_mav_time_ms);
			test_f = (test_f * 2 * 3.1415926) / test_mav_time_ms;
			test_f = sin(test_f);
			test_f *= test_mav_value;
			r_pc_mesg.cmd[0] = 4;
			r_pc_mesg.cmd[1] = 0;
			r_pc_mesg.cmd[2] = 4;

			r_pc_mesg.params[0].fdata = (float)test_f;
			r_pc_mesg.params[1].fdata = (float)test_f;
			r_pc_mesg.params[2].fdata = (float)test_f;
			Deal_Motor_Cmd_Custom_485(&Motor_Devive[0],&r_pc_mesg);
		}
		


#else
		poll_ret = poll(fds, 1, 500);
		test_motor_flag = 0;
		if (poll_ret == 0) 
		{
			//printf("Got no data within a second\n");

		}
		else if (poll_ret < 0)
		{
			if (error_counter < 10 || error_counter % 50 == 0) 
			{
				syslog(LOG_ERR, "motor_contrl: ERROR return value from poll(): %d\n", poll_ret);
			}
			error_counter++;
		}
		else 
		{
			orb_check(pc_mesg_sub_fd,&update);
			if(update)
			{
				//time_now = hrt_absolute_time();
				test_motor_flag = 1;
				orb_copy(ORB_ID(pc_motor_cmd_uorb), pc_mesg_sub_fd, &r_pc_mesg);
				test_motor_flag = 2;
				t_motor_state_mesg.mask = 0;
				for (i=0;i<APP_MAX_MOTOR_NO;i++)
				{
					if (r_pc_mesg.cmd[i] == 0)
					{
						continue;
					}
					else if (r_pc_mesg.cmd[i] < MOTOR_PWM_CMD_START)
					{
						ret = Deal_Motor_Cmd_Elmo(params, state, &Motor_Devive[0], &r_pc_mesg,motor_pwm_param, (unsigned char)i);
					}
					else 
					{
						ret = Deal_Motor_pwm_Cmd_Elmo(motor_pwm_param,&Motor_Devive[0],&r_pc_mesg,i);
					}
					if ((ret != 0) || (r_pc_mesg.state == MOTOR_CMD_STATE_ERR))
					{
						syslog(LOG_WARNING,"motor_control:Deal PC Motor Cmd err %d\n",ret);

						t_motor_state_mesg.params[i].str[0] = ret;
						t_motor_state_mesg.params[i].str[1] = 0;
						t_motor_state_mesg.state[i] = MOTOR_CMD_STATE_ERR;
						t_motor_state_mesg.cmd[i] = r_pc_mesg.cmd[i];
						t_motor_state_mesg.mask |= 0x01<<i;
						r_pc_mesg.cmd[i] = 0;
					}
					else
					{
						if (r_pc_mesg.state == MOTOR_CMD_STATE_COMPLETE)
						{
							t_motor_state_mesg.params[i].u_intdata = r_pc_mesg.params[i].u_intdata;
							t_motor_state_mesg.state[i] = MOTOR_CMD_STATE_COMPLETE;
							t_motor_state_mesg.cmd[i] = r_pc_mesg.cmd[i];
							t_motor_state_mesg.mask |= 0x01<<i;
						}
					} 
					test_motor_flag = 4;
				}
				if (t_motor_state_mesg.mask)
				{
					orb_publish(ORB_ID(motor_state_uorb), att_motor_state_pub, &t_motor_state_mesg);
					memset(&t_motor_state_mesg, 0, sizeof(t_motor_state_mesg));
					usleep(20000);
				}
			}
		}
#endif
	}
  /* Free/uninitialize data structures */
  //return NULL;
}







