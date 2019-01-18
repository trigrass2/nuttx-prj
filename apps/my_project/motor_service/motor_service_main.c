/****************************************************************************
 * examples/canopen/main.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************
 * Leveraged from:
 *
 *   Freecanopen Libary: Linux Demo Application
 *   Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

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
#include <motor_service/elmo_app.h>
#include <motor_service/motor_fppa.h>
#include "uORB/topic/motor_state_uorb.h"

//#include "nuttx/arch/arm/"
//#include "stm32_gpio.h"

#include "uORB/uorb/uORB.h"
///////////////////////////////////////////////////////////////////////
//test code
extern int test_mav_time_ms;
extern int test_mav_control_t;
extern int test_mav_value;
/////////////////////////////////////////////////////////////////////////////
//extern unsigned int test_temp;

#define CANOPEN_MOTOR_1
#define CANOPEN_MOTOR_2
#define CANOPEN_MOTOR_3
//#define CANOPEN_MOTOR_4
//#define CANOPEN_MOTOR_5
//#define CANOPEN_MOTOR_6

#define MOTOR_UART_DEVICE	"/dev/ttyS4"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/



/****************************************************************************
 * Private Types
 ****************************************************************************/



enum canopen_threadstate_e
{
  STOPPED = 0,
  RUNNING,
  SHUTDOWN
};

int watchdog_elmo;

struct Motor_Ttask_Struct Motor_Ttask_s;

FPPA_MOTOR_DEVICE Motor_Devive[DEVICE_MAX_MOTOR_NO];

struct motor_state_uorb_s t_motor_state_mesg;
orb_advert_t att_motor_state_pub;
struct motor_state_uorb_s motor_position_state_mesg;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void 		canopen_showusage(FAR const char *progname, int exitcode);
void 	*canopen_tx_pollthread(void *pvarg);
void canopen_tx_fun(void *pvarg);
void 	*canopen_network_pollthread(void *pvarg);
void 	*canopen_terminal_pollthread(void *pvarg);

static int motor_service_task(int argc, char *argv[]);
static void *canopen_rx_pollthread(void *pvarg);

void get_drv_info(uint8_t id);
int Deal_Motor_Uart_Read(uint8_t *buf,int data_long);
/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: canopen_initialize
 *
 * Description:
 *   Called from the canopen polling thread in order to initialized the
 *   Freecanopen interface.
 *
 ****************************************************************************/

/*
 ------------------------------------------------------------------------------------------------
 -	received thread programs
 - ----------------------------------------------------------------------------------------------
 */

/****************************************************************************
 * Name: canopen_rx_pollthread
 *
 * Description:
 *   This is the canopen receive thread.
 *
 ****************************************************************************/
static void *canopen_rx_pollthread(void *pvarg)
{
	int ret;
	 /* set the timer attributes and start the timer */
	ret = timerDeviceConfigration();
	//uint8_t	uart_rx_buf[MOTOR_UART_BUF_RX_SIZE];
	if(ret <  0)
	{
		fprintf(stderr, "timerConfigration: "
					  "ERROR: timer configure failed: %d\n", ret);
	}

	//memset(&t_motor_back_data_mesg, 0, sizeof(t_motor_back_data_mesg));
	//motor_back_data_uorb_pub = orb_advertise_queue(ORB_ID(motor_back_data_uorb), &t_motor_back_data_mesg,DEVICE_MAX_MOTOR_NO);

	

	do
	{
#ifdef	CONFIG_MOTOR_DRIVER_ELMO
		canDeviceMsgRead(0);
#else
		/*get_drv_info(10);
		usleep(5000);
		Deal_Motor_Uart_Read(&uart_rx_buf[0],MOTOR_UART_BUF_RX_SIZE);
		get_drv_info(11);
		usleep(5000);
		Deal_Motor_Uart_Read(&uart_rx_buf[0],MOTOR_UART_BUF_RX_SIZE);
		get_drv_info(12);
		usleep(5000);
		Deal_Motor_Uart_Read(&uart_rx_buf[0],MOTOR_UART_BUF_RX_SIZE);
		*/
		sleep(10);		 
#endif
	}
	while (Motor_Ttask_s.task_state != MOTOR_TASK_STATE_EXIT);
	return NULL;
}


static int motor_service_task(int argc, char *argv[])
{
  int ret = -1;
  int i;
  pthread_t thread_motor_rx;

  //stm32_configgpio(GPIO_BUZZER);

  /* open USART port */
#ifdef CONFIG_MOTOR_DRIVER_CUSTOM_485
	//ret = open(MOTOR_UART_DEVICE, O_RDWR | O_NONBLOCK);
	ret = open(MOTOR_UART_DEVICE, O_RDWR);
	if (ret <  0)
	{
		syslog(LOG_ERR,"motor_service:ERROR: motor uart device open failed: %d\n", ret );
		return 0;
	}
	else
	{
		syslog(LOG_INFO,"motor_service:motor uart device open ok!\n");
		for (i = 0; i < APP_MAX_MOTOR_NO; i++)
		{
			Motor_Devive[i].uart_fd = ret;
		}
	}
#endif
#ifdef CONFIG_MOTOR_DRIVER_ELMO
	/* open canopen can port */
	ret = canDeviceOpen(0);
	if(ret <  0){
		fprintf(stderr, "CanPortInit: "
					  "ERROR: CanPortInit failed: %d\n", ret);
	}

	/* open canopen timer port */
	ret = timerDeviceOpen();
	if(ret <  0){
		fprintf(stderr, "timerPortInit: "
							  "ERROR: timerPortInit failed: %d\n", ret);
		return 0;
	}
	
	
	/* create the canopen  rx thread*/

	pthread_attr_t g_canopen_rx_pthread_attr ={
		  .priority 	= CONFIG_MOTOR_READ_THREAD_PRIORITY,
		  .policy		= PTHREAD_DEFAULT_POLICY,
		  .inheritsched = PTHREAD_EXPLICIT_SCHED,
		  .stacksize	= PTHREAD_STACK_DEFAULT
	};
	
	if (Motor_Ttask_s.task_state != MOTOR_TASK_STATE_EXIT){
	  ret = pthread_create(&thread_motor_rx,\
						   &g_canopen_rx_pthread_attr,\
						   canopen_rx_pollthread,\
						   NULL);
	}else{
	  fprintf(stderr, "pthread_create: "
			  "ERROR: send pthread create failed: %d\n", ret);
	  return 0;
	}


	/* open PWM port */


	Motor_Devive[0].pwm_fd = open("/dev/motor_pwm1", O_RDWR);
	if (Motor_Devive[0].pwm_fd <  0)
	{
		fprintf(stderr, "motor_service: "
				"ERROR: /dev/motor_pwm1 open failed: %d\n", Motor_Devive[0].canopen_fd );
		return 0;
	}

  /* open motor1 port */
	Motor_Devive[0].canopen_fd = open("/dev/motor1", O_RDWR);
	if (Motor_Devive[0].canopen_fd <  0)
	{
		fprintf(stderr, "canopen_initialize: "
				"ERROR: /dev/motor1 open failed: %d\n", Motor_Devive[0].canopen_fd );
		return 0;
	}
	else
	{
		Motor_Devive[0].canopen_id = ioctl(Motor_Devive[0].canopen_fd, PWRIOC_GET_DEVID, 0);
	}
	if (APP_MAX_MOTOR_NO > 1)
	{
		// open motor2 port
		Motor_Devive[1].canopen_fd = open("/dev/motor2", O_RDWR);
		if (Motor_Devive[1].canopen_fd <  0)
		{
			fprintf(stderr, "canopen_initialize: "
					"ERROR: /dev/motor2 open failed: %d\n", Motor_Devive[1].canopen_fd );
			return 0;
		}
		else
		{
			Motor_Devive[1].canopen_id = ioctl(Motor_Devive[1].canopen_fd, PWRIOC_GET_DEVID, 0);
			Motor_Devive[1].pwm_fd = Motor_Devive[0].pwm_fd;
		}
	}
	if (APP_MAX_MOTOR_NO > 2)
	{
		// open motor3 port 
		Motor_Devive[2].canopen_fd = open("/dev/motor3", O_RDWR);
		if (Motor_Devive[2].canopen_fd <  0)
		{
			fprintf(stderr, "canopen_initialize: "
					"ERROR: /dev/motor3 open failed: %d\n", Motor_Devive[2].canopen_fd );
			return 0;
		}
		else
		{
			Motor_Devive[2].canopen_id = ioctl(Motor_Devive[2].canopen_fd, PWRIOC_GET_DEVID, 0);
			Motor_Devive[2].pwm_fd = Motor_Devive[0].pwm_fd;
		}

	}

	// open motor4 port 
	if (APP_MAX_MOTOR_NO > 3)
	{
		Motor_Devive[3].canopen_fd = open("/dev/motor4", O_RDWR);
		if (Motor_Devive[3].canopen_fd <  0)
		{
			fprintf(stderr, "canopen_initialize: "
					"ERROR: /dev/motor4 open failed: %d\n", Motor_Devive[3].canopen_fd );
			return 0;
		}
		else
		{
			Motor_Devive[3].canopen_id = ioctl(Motor_Devive[3].canopen_fd, PWRIOC_GET_DEVID, 0);
		}
	}
	if (APP_MAX_MOTOR_NO > 4)
	{
		// open motor5 port 
		Motor_Devive[4].canopen_fd = open("/dev/motor5", O_RDWR);
		if (Motor_Devive[4].canopen_fd <  0)
		{
			fprintf(stderr, "canopen_initialize: "
					"ERROR: /dev/motor5 open failed: %d\n", Motor_Devive[4].canopen_fd );
			return 0;
		}
		else
		{
			Motor_Devive[4].canopen_id = ioctl(Motor_Devive[4].canopen_fd, PWRIOC_GET_DEVID, 0);
		}

	}
	if (APP_MAX_MOTOR_NO > 5)
	{
		// open motor6 port 
		Motor_Devive[5].canopen_fd = open("/dev/motor6", O_RDWR);
		if (Motor_Devive[5].canopen_fd <  0)
		{
			fprintf(stderr, "canopen_initialize: "
					"ERROR: /dev/motor6 open failed: %d\n", Motor_Devive[5].canopen_fd );
			return 0;
		}
		else
		{
			Motor_Devive[5].canopen_id = ioctl(Motor_Devive[5].canopen_fd, PWRIOC_GET_DEVID, 0);
		}
	}
#endif





  /* Then loop until we are commanded to shutdown */

	//stm32_gpiowrite(GPIO_BUZZER, 1);
	//sleep(1);
	//stm32_gpiowrite(GPIO_BUZZER, 0);

	memset(&t_motor_state_mesg, 0, sizeof(t_motor_state_mesg));
	att_motor_state_pub = orb_advertise(ORB_ID(motor_state_uorb), &t_motor_state_mesg);
	memset(&motor_position_state_mesg, 0, sizeof(motor_position_state_mesg));

	canopen_tx_fun(NULL);

  /* Disable */


  /* Release hardware resources. */
#ifdef CONFIG_MOTOR_DRIVER_ELMO
  canDeviceClose();

  timerDeviceClose();
  for (i = 0; i < DEVICE_MAX_MOTOR_NO; i++)
  {
	  if (Motor_Devive[i].canopen_fd > 0)
	  {
		  close(Motor_Devive[i].canopen_fd);
	  }
  }

  /* Free/uninitialize data structures */

  //(void)pthread_mutex_destroy(&rx_thread.lock);
#endif
#ifdef CONFIG_MOTOR_DRIVER_CUSTOM_485
  close(Motor_Devive[0].uart_fd);
#endif

  return 0;
}


/****************************************************************************
 * Name: canopen_showusage
 *
 * Description:
 *   Show usage of the demo program and exit
 *
 ****************************************************************************/

static void canopen_showusage(FAR const char *progname, int exitcode)
{
  printf("USAGE: %s [-d|e|s|q|h]\n\n", progname);
  printf("Where:\n");
  printf("  -e : Enable the protocol stack\n");
  printf("  -q : Quit application\n");
  printf("  -h : Show this information\n");
  printf("\n");
  exit(exitcode);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: canopen_main
 *
 * Description:
 *   This is the main entry point to the demo program
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int motor_service_main(int argc, char *argv[])
#endif
{
  int option;
  int ret;

  /* Handle command line arguments */

  //g_canopen.quit = false;

  while ((option = getopt(argc, argv, "ceqhtv")) != -1)
  {
      switch (option){
		  case 'c':
			test_mav_control_t = atoi(argv[2]);
			test_mav_control_t >>= 1;
			  break;
          case 'e': /* Enable the protocol stack */
            {
				if (Motor_Ttask_s.task_state != MOTOR_TASK_STATE_RUN)
				{
					Motor_Ttask_s.task_state = MOTOR_TASK_STATE_RUN;
					  ret =  task_create("motor_service", CONFIG_MOTOR_TASK_PRIORITY,3072,motor_service_task, NULL);
					  if (ret < 0){
						  syslog(LOG_INFO, "motor_service_main: ERROR: creat motor_service task failed: %d\n", ret);
						  return EXIT_FAILURE;
					  }
				}
				else
				{
					syslog(LOG_INFO, "motor_service_main: the task has runing%d\n", ret);
				}

            }
            break;
          case 'q': /* Quit application */
            //g_canopen.quit = true;
			Motor_Ttask_s.task_state = MOTOR_TASK_STATE_EXIT;

            break;

          case 'h': /* Show help info */
            canopen_showusage(argv[0], EXIT_SUCCESS);
            break;
		  case 't':
			  test_mav_time_ms = atoi(argv[2]);
		  break;
		  case 'v':
			  test_mav_value = atoi(argv[2]);
		  break;
	  

		  default:
		  {
			printf("canopen_main: Unrecognized option: '%c'\n", option);
			canopen_showusage(argv[0], EXIT_FAILURE);
			break;
		  }
      	 }
	}
  //exit(EXIT_FAILURE);
  return EXIT_SUCCESS;
}


void Motor_Deal_Back_Data(int32_t str[], unsigned int module_id,unsigned char data_type)
{
	FAR struct motor_state_s motor_state;
	int temp_uint;
	int ret;
	int no;
	float temp_f;
	
	for (no = 0;no < APP_MAX_MOTOR_NO;no++)
	{
		if (Motor_Devive[no].canopen_id == module_id)
		{
			break;
		}
	}
	if (no >= APP_MAX_MOTOR_NO)
	{
		return;
	}
	if (Motor_Devive[no].canopen_fd > 0)
	{
		if (str[0] == 1)//position
		{

				temp_uint = str[1];
				temp_uint = temp_uint % MOTOR_LINE_ONECIRCLE;
				if (temp_uint < 0)
				{
					temp_uint = MOTOR_LINE_ONECIRCLE + temp_uint;
				}
				temp_f = (float)temp_uint;
				temp_f = temp_f * (360.0/MOTOR_LINE_ONECIRCLE);//convert line to angle
				motor_state.fb.position = temp_f;

				temp_f -= Motor_Devive[no].zero_position;
				while (temp_f < 0)
				{
					temp_f += 360;
				}
				while (temp_f >= 360)
				{
					temp_f -= 360;
				}
				watchdog_elmo = 0;
				//t_motor_back_data_mesg.params.fdata = temp_f;
				if (data_type == ELMO_RETURN_TIMER)
				{
					motor_position_state_mesg.params[no].fdata = temp_f;
					motor_position_state_mesg.cmd[no] = MOTOR_BACK_CMD_TIMER_BACK;
					motor_position_state_mesg.state[no] = MOTOR_CMD_STATE_DO;
					motor_position_state_mesg.mask |= 0x01<<no;
					if (motor_position_state_mesg.mask == (0XFF >> (8-APP_MAX_MOTOR_NO)))
					{
						memcpy(&t_motor_state_mesg,&motor_position_state_mesg,sizeof(t_motor_state_mesg));
						memset(&motor_position_state_mesg, 0, sizeof(motor_position_state_mesg));
					}
					motor_state.state = MOTOR_STATE_RUN;
				}
				else
				{
					//t_motor_back_data_mesg.cmd = MOTOR_BACK_CMD_POSITION_OK;
					//t_motor_back_data_mesg.state = MOTOR_CMD_STATE_COMPLETE;
					t_motor_state_mesg.params[no].fdata = temp_f;
					t_motor_state_mesg.state[no] = MOTOR_CMD_STATE_COMPLETE;
					t_motor_state_mesg.cmd[no] = MOTOR_BACK_CMD_POSITION_OK;
					t_motor_state_mesg.mask |= 0x01<<no;

					motor_state.state = MOTOR_STATE_IDLE;
				}
				ret = ioctl(Motor_Devive[no].canopen_fd, PWRIOC_SET_STATE, (unsigned long)&motor_state);
				if (ret < 0)
				{
					syslog(LOG_ERR, "mostor_service_main: ERROR ioctl fail: %d\n", ret);
				}
				if (t_motor_state_mesg.mask > 0)
				{
					orb_publish(ORB_ID(motor_state_uorb), att_motor_state_pub, &t_motor_state_mesg);
					t_motor_state_mesg.mask = 0;
				}
		}
	}
}




				



 
