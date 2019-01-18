//#ifdef CONFIG_MOTOR_DRIVER_ELMO

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

#include <nuttx/power/elmo_db4x_pwm_io.h>
extern FPPA_MOTOR_DEVICE Motor_Devive[DEVICE_MAX_MOTOR_NO];
extern struct motor_state_uorb_s t_motor_state_mesg;
extern orb_advert_t att_motor_state_pub;

int Search_Zero_Speed(struct motor_params_s motor_s[],struct motor_state_s state_s[],
				FPPA_MOTOR_DEVICE motor_fds[],struct motor_pwm_io_param_s temp_pwm[],
				unsigned char motor_no);



int Deal_Motor_Cmd_Elmo(struct motor_params_s motor_s[],
				   struct motor_state_s state_s[],
				   FPPA_MOTOR_DEVICE motor_dev_fd[],
				   struct pc_motor_cmd_uorb_s *contrl_cmd,
				   struct motor_pwm_io_param_s temp_pwm[],
				   unsigned char motor_no)
{
	int ret,err;
	uint32_t temp;
	float temp_f;

	//float zero_position[8];
	//static char flag_zero;

	err = 0;
	if (motor_no >= APP_MAX_MOTOR_NO)
	{
		err = DO_CMD_ERR_OUTRANGE;
		return err;
	}
	if (motor_dev_fd[motor_no].zero_position >  LIMIT_POSITION_ANGLE_HARDWARE)
	{
		if ((contrl_cmd->cmd[motor_no] != 3) && 
			(contrl_cmd->cmd[motor_no] != 17) &&
			(contrl_cmd->cmd[motor_no] != 14) )//just can serch zreo
		{
			err = DO_CMD_ERR_ILLICIT;
			return err;
		}
	}
	if (motor_dev_fd[motor_no].canopen_fd <= 0)
	{
		err = DO_CMD_ERR_FILE;
			return err;
	}
	switch (contrl_cmd->cmd[motor_no])
		{
		case 0:
			break;
		case 1://read state
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd, PWRIOC_GET_STATE, (unsigned long)&state_s[motor_no]);
			if (ret >= 0)
			{
				contrl_cmd->params[motor_no].u_intdata = state_s[motor_no].state;
				contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
				//ret = mq_send(g_send_mqfd, &float_int_data.str[0], 4, 42);
			}
			else 
			{
				contrl_cmd->state = MOTOR_CMD_STATE_ERR;
				err = DO_CMD_ERR_FILE;
			}
			break;
		case 16://stop
			motor_dev_fd[motor_no].control_type = MOTOR_OPMODE_SHUTDOWN;
			temp = motor_s[motor_no].speed;
			motor_s[motor_no].speed = 0;//stop
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_MODE, MOTOR_OPMODE_SHUTDOWN);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			motor_s[motor_no].speed = temp;
		break;
		case 2://restart
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_MODE, MOTOR_OPMODE_START);
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret < 0)
			{
				err = DO_CMD_ERR_FILE; 
			}
			//ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd, PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret >= 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_DO;
				motor_dev_fd[motor_no].control_type = MOTOR_OPMODE_STOP;
			}
			else
			{
				err = DO_CMD_ERR_FILE;
			}
			break;
		case 17://search zero position
			//if(Search_Zero(&motor_s[0],&state_s[0],&Motor_Devive[0],&zero_position[0],APP_MAX_MOTOR_NO) == 0)
			if(Search_Zero_Speed(&motor_s[0],&state_s[0],&Motor_Devive[0],temp_pwm,APP_MAX_MOTOR_NO) == 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
				//flag_zero = 1;
			}
			else
			{	
				contrl_cmd->state = MOTOR_CMD_STATE_ERR;
				err = 3;
			}
			break;
		case 3://zero position

			for (ret = 0;ret < APP_MAX_MOTOR_NO;ret++)
			{
				if (motor_dev_fd[ret].zero_position <=  LIMIT_POSITION_ANGLE_HARDWARE)
				{
					break;
				}
			}
			if (ret >=  APP_MAX_MOTOR_NO)
			{
				//if(Search_Zero(&motor_s[0],&state_s[0],&Motor_Devive[0],&zero_position[0],APP_MAX_MOTOR_NO) == 0)
				if(Search_Zero_Speed(&motor_s[0],&state_s[0],&Motor_Devive[0],temp_pwm,APP_MAX_MOTOR_NO) == 0)
				{
					contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
					//flag_zero = 1;
				}
				else
				{	
					contrl_cmd->state = MOTOR_CMD_STATE_ERR;
					err = 3;
				}
			}
			else
			{
				if (motor_dev_fd[motor_no].zero_position >  LIMIT_POSITION_ANGLE_HARDWARE)
				{
					contrl_cmd->state = MOTOR_CMD_STATE_ERR;
					err = DO_CMD_ERR_ILLICIT;
					return err;
				}
				if (motor_dev_fd[motor_no].control_type != MOTOR_OPMODE_POSITION_ABSOLUTE)
				{
					motor_dev_fd[motor_no].control_type = MOTOR_OPMODE_POSITION_ABSOLUTE;
					ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_MODE, MOTOR_OPMODE_POSITION_ABSOLUTE);
					if (ret < 0)
					{
					  err = DO_CMD_ERR_FILE;
					}
					ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
					ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[motor_no]);
					if (ret < 0)
					{
					  err = DO_CMD_ERR_FILE;
					}

				}


				motor_s[motor_no].position = motor_dev_fd[motor_no].zero_position;

				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
				if (ret < 0)
				{
				  err = DO_CMD_ERR_FILE;
				}
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_POSITIN, (unsigned long)&motor_s[motor_no]);
				if (ret >= 0)
				{
					contrl_cmd->state = MOTOR_CMD_STATE_DO;
				}
				else
				{
					err = DO_CMD_ERR_FILE;
				}
				state_s[motor_no].state  = MOTOR_STATE_RUN;
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd, PWRIOC_SET_STATE, (unsigned long)&state_s[motor_no]);
			}
			break;
		case 4://ABSOLUTE position
			if ((contrl_cmd->params[motor_no].fdata > LIMIT_ELMO_POSITION_ANGLE_SOFTWARE) || 
				(contrl_cmd->params[motor_no].fdata < -LIMIT_ELMO_POSITION_ANGLE_SOFTWARE))
			{

				err = DO_CMD_ERR_OUTRANGE;
				break;
			}
			if (motor_dev_fd[motor_no].control_type != MOTOR_OPMODE_POSITION_ABSOLUTE)
			{
				motor_dev_fd[motor_no].control_type = MOTOR_OPMODE_POSITION_ABSOLUTE;
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_MODE, MOTOR_OPMODE_POSITION_ABSOLUTE);
				if (ret < 0)
				{
				  err = DO_CMD_ERR_FILE;
				}
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[motor_no]);
				if (ret < 0)
				{
				  err = DO_CMD_ERR_FILE;
				}
				

			}
			motor_s[motor_no].position = contrl_cmd->params[motor_no].fdata + motor_dev_fd[motor_no].zero_position;

			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS,(unsigned long)&motor_s[motor_no]);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_POSITIN, (unsigned long)&motor_s[motor_no]);
			if (ret >= 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_DO;
				contrl_cmd->params[motor_no].fdata = motor_s[motor_no].position;
				if (contrl_cmd->params[motor_no].fdata < 0)
				{
					contrl_cmd->params[motor_no].fdata += 360;
				}
			}
			else
			{
				err = DO_CMD_ERR_FILE;
			}
			state_s[motor_no].state  = MOTOR_STATE_RUN;
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd, PWRIOC_SET_STATE, (unsigned long)&state_s[motor_no]);
			break;
		case 5://read position

			temp = 0;
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd, PWRIOC_SEND_SYNC, (unsigned long)&temp);
			if (ret >= 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_DO;
			}
			else 
			{
				err = DO_CMD_ERR_FILE;
				contrl_cmd->state = MOTOR_CMD_STATE_ERR;
			}
			break;
		case 6://set speed
			if ((contrl_cmd->params[motor_no].fdata > MAX_ELMO_VELOCITY_ANGLE) || 
				(contrl_cmd->params[motor_no].fdata < 0))
			{

				err = DO_CMD_ERR_OUTRANGE;
				break;
			}
			motor_s[motor_no].speed = contrl_cmd->params[motor_no].fdata;
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret >= 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
				contrl_cmd->params[motor_no].fdata = motor_s[motor_no].speed;
			}
			else 
			{
				err = DO_CMD_ERR_FILE;
			}
			break;
		case 7://read speed
			contrl_cmd->params[motor_no].fdata = motor_s[motor_no].speed;
			contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
			break;
		case 8://set motor acc
			if ((contrl_cmd->params[motor_no].fdata > MAX_ELMO_ACCELERATE_ANGLE) || 
				(contrl_cmd->params[motor_no].fdata < 0))
			{
				err = DO_CMD_ERR_OUTRANGE;
				break;
			}
			motor_s[motor_no].acceleration = contrl_cmd->params[motor_no].fdata;
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret >= 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
				contrl_cmd->params[motor_no].fdata = motor_s[motor_no].acceleration;
			}
			else
			{
				err = DO_CMD_ERR_FILE;
			}
			break;
		case 9://read acc
			contrl_cmd->params[motor_no].fdata = motor_s[motor_no].acceleration;
			contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
			break;
		case 10://set dec
			if ((contrl_cmd->params[motor_no].fdata > MAX_ELMO_ACCELERATE_ANGLE) || 
				(contrl_cmd->params[motor_no].fdata < 0))
			{
				err = DO_CMD_ERR_OUTRANGE;
				break;
			}
			motor_s[motor_no].deceleartion = contrl_cmd->params[motor_no].fdata;
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret >= 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
				contrl_cmd->params[motor_no].fdata = motor_s[motor_no].deceleartion;
			}
			else
			{
				err = DO_CMD_ERR_FILE;
			}
			break;
		case 11://read dec
			contrl_cmd->params[motor_no].fdata = motor_s[motor_no].deceleartion;
			contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
			break;
		case 12://set relative position
			if ((contrl_cmd->params[motor_no].fdata > LIMIT_ELMO_POSITION_ANGLE_SOFTWARE) || 
				(contrl_cmd->params[motor_no].fdata < -LIMIT_ELMO_POSITION_ANGLE_SOFTWARE))
			{
				err = DO_CMD_ERR_OUTRANGE;
				break;
			}
			if (motor_dev_fd[motor_no].control_type != MOTOR_OPMODE_POSITION_RELATIVE)
			{
				motor_dev_fd[motor_no].control_type = MOTOR_OPMODE_POSITION_RELATIVE;
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_MODE, MOTOR_OPMODE_POSITION_RELATIVE);
				if (ret < 0)
				{
				  err = DO_CMD_ERR_FILE;
				}
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)&motor_s[motor_no]);
				if (ret < 0)
				{
				  err = DO_CMD_ERR_FILE;
				}

			}
			
			motor_s[motor_no].position = contrl_cmd->params[motor_no].fdata;

			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_PARAMS, (unsigned long)&motor_s[motor_no]);
			if (ret < 0)
			{
			  err = DO_CMD_ERR_FILE;
			}
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_POSITIN, (unsigned long)&motor_s[motor_no]);
			if (ret >= 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_DO;

				contrl_cmd->params[motor_no].fdata = state_s[motor_no].fb.position - motor_dev_fd[motor_no].zero_position + motor_s[motor_no].position;
				if (contrl_cmd->params[motor_no].fdata < 0)
				{
					contrl_cmd->params[motor_no].fdata += 360;
				}
			}
			else
			{
				err = DO_CMD_ERR_FILE;
			}
			state_s[motor_no].state  = MOTOR_STATE_RUN;
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd, PWRIOC_SET_STATE, (unsigned long)&state_s[motor_no]);
			break;
		case 15://read err code
			contrl_cmd->params[motor_no].u_intdata = err;
			contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
			break;
		case 14://read memory position
			temp = motor_no + 1;
			ret = ioctl(motor_dev_fd[motor_no].canopen_fd, PWRIOC_SEND_SYNC, (unsigned long)&temp);
			if (ret >= 0)
			{
				contrl_cmd->state = MOTOR_CMD_STATE_DO;
				temp_f = (float)temp;
				temp_f = temp_f * (360.0/MOTOR_LINE_ONECIRCLE);//convert line to angle
				if ((temp_f < LIMIT_POSITION_ANGLE_HARDWARE) && (temp_f > -LIMIT_POSITION_ANGLE_HARDWARE))
				{
					//zero_position[motor_no] = temp_f;
				}
				else
				{
					err = DO_CMD_ERR_FAULT;
					syslog(LOG_ERR, "motor_control: read back zero position is out of range: %f\n", temp_f);
				}
				
			}
			else 
			{
				err = DO_CMD_ERR_FILE;
				contrl_cmd->state = MOTOR_CMD_STATE_ERR;
			}
		break;
		case 13:
		default:
			contrl_cmd->state = MOTOR_CMD_STATE_ERR;
			err = DO_CMD_ERR_CMDNODEFINE;
			//ret = mq_send(g_send_mqfd, &float_int_data.str[0], 4, 42);
			break;
		}
	return err;
}


//=====================================================================
//fun name:Deal_Motor_pwm_Cmd_Elmo
//return :
//=====================================================================

int Deal_Motor_pwm_Cmd_Elmo(struct motor_pwm_io_param_s temp_pwm[], 
				 FPPA_MOTOR_DEVICE motor_dev_fd[],
				 struct pc_motor_cmd_uorb_s *contrl_cmd,
				   int motor_no)
{
	int ret,err;
	unsigned char timer_no;
	float temp_f;
 	unsigned char pwm_chanal;
	

	err = 0;
	if ((motor_no >= APP_MAX_MOTOR_NO) || (motor_no < 0))
	{
		err = DO_CMD_ERR_ILLICIT;
		return err;
	}
	if ((motor_dev_fd[motor_no].canopen_fd <= 0) ||
		(motor_dev_fd[motor_no].pwm_fd <= 0))
	{
		err = DO_CMD_ERR_FILE;
			return err;
	}
	if (motor_no >= 3)
	{
		pwm_chanal = motor_no - 3;
		timer_no = 1;
	}
	else
	{
		timer_no = 0;
		pwm_chanal = motor_no;
	}

	if(contrl_cmd->cmd[motor_no] == 20)
	{
		temp_f =  contrl_cmd->params[motor_no].fdata;
		if ((temp_f > 100.001) || (temp_f < -100.001))
		{
			syslog(LOG_ERR, "motor_contrl: pwm control cmd percent is out of range: %d\n", temp_f);
			err = DO_CMD_ERR_ILLICIT;
		}
		else
		{

			if (motor_dev_fd[motor_no].control_type != MOTOR_OPMODE_SPEED)
			{
				motor_dev_fd[motor_no].control_type = MOTOR_OPMODE_SPEED;
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_SET_MODE, MOTOR_OPMODE_SPEED);
				if (ret < 0)
				{
				  err = DO_CMD_ERR_FILE;
				}
				ret = ioctl(motor_dev_fd[motor_no].canopen_fd,PWRIOC_RUN_PARAMS, (unsigned long)NULL);
				if (ret < 0)
				{
				  err = DO_CMD_ERR_FILE;
				}

			}
			temp_f = (temp_f * PWM_PERCENT_MAX) / 100.0;
			temp_pwm[timer_no].duties[pwm_chanal] = (int)temp_f;
			ret = ioctl(motor_dev_fd[motor_no].pwm_fd, PWRIOC_RUN_SPEED, (unsigned long)((uintptr_t)&temp_pwm[timer_no]));
			
			if (ret < 0)
			{
				err = 2;
			}
			else
			{
				contrl_cmd->state = MOTOR_CMD_STATE_DO;
			}
		}
	}
	else if(contrl_cmd->cmd[motor_no] == 21)
	{
		temp_pwm[timer_no].frequency = contrl_cmd->params[motor_no].u_intdata;
		if ((temp_pwm[timer_no].frequency > PWM_MAX_FREQUENCY) || (temp_pwm[timer_no].frequency < PWM_MIN_FREQUENCY))
		{
			syslog(LOG_ERR, "motor_contrl: pwm control cmd frequency is out of range: %d\n", temp_pwm[timer_no].frequency);
			err = DO_CMD_ERR_ILLICIT;
		}
		else
		{
			ret = ioctl(motor_dev_fd[motor_no].pwm_fd, PWRIOC_RUN_SPEED, (unsigned long)((uintptr_t)&temp_pwm[timer_no]));
			if (ret < 0)
			{
				err = 21;
			}
			else
			{
				contrl_cmd->state = MOTOR_CMD_STATE_COMPLETE;
			}
		}
	}
	else 
	{
		err = DO_CMD_ERR_CMDNODEFINE;
	}
	return err;
}

