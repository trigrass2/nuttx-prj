/**
  ******************************************************************************
  * @file    can_stm32.c
  * @author  Zhenglin R&D Driver Software Team
  * @version V1.0.0
  * @date    26/04/2015
  * @brief   This file is can_stm32 file.
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include "canopen_stack/canfestival.h"
#include "canopen_stack/can_device.h"
#include <nuttx/timers/timer.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
// Store the last timer value to calculate the elapsed time
static TIMEVAL last_time_set = CONFIG_CANOPEN_TIMER_INTERVAL;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Configuration ************************************************************/

#define TIMER_PATH 	"/dev/timer6"
static int      timer_fd = -1;
//unsigned int test_temp;

/**
  * @brief  setTimer
  * @param  value:Set time value 0x0000-0xffff
  * @retval NONE
  */
void setTimer(TIMEVAL value)
{
	int ret = 0;

	ret = ioctl(timer_fd, TCIOC_SETPERIOD, value);
	if (ret < 0)
	{
		syslog(LOG_ERR,"ioctl timer err！\n");
	}
}

uint32_t timerDeviceGetCanopenCount(void)
{
	struct timer_status_s status;
	int ret;

	/* Get timer status */

	ret = ioctl(timer_fd, TCIOC_GETSTATUS, (unsigned long)((uintptr_t)&status));
	if (ret  > 0)
	{
		return status.timeleft;
	}
	else
	{
		return 0;
	}
}
/**
  * @brief  getElapsedTime
  * @param  NONE
	* @retval TIMEVAL:Return current timer value
  */
TIMEVAL getElapsedTime(void)
{
	
	uint32_t timer;
	timer = timerDeviceGetCanopenCount();
	return timer > last_time_set ? timer - last_time_set : last_time_set - timer;
}

/**
  * @brief  timerDeviceHandler
  * @param  NONE
  * @retval NONE
  */
static void timerDeviceHandler(int signo, FAR siginfo_t *siginfo,
                             FAR void *context)
{
	last_time_set = timerDeviceGetCanopenCount();
	TimeDispatch();
	//test_temp++;
}


/****************************************************************************
 * Name: timerDeviceOpen
 *
 * Description:
 *   open timer port and return fd.
 *
 ****************************************************************************/
int timerDeviceOpen()
{

	/* Open the timer device */

	timer_fd = open(TIMER_PATH, O_RDONLY);
	if (timer_fd < 0){
		syslog(LOG_ERR,"open timer err！\n");
		return timer_fd;
	}
	return OK;
}

/****************************************************************************
 * Name: timerDeviceConfigration
 *
 * Description:
 *   configure soft timer.
 *
 ****************************************************************************/
int timerDeviceConfigration()
{
	struct timer_notify_s notify;
	struct sigaction act;
	int ret;

	/* Set the timer interval */

	//printf("Set timer interval to %lu\n",(unsigned long)10000);

	ret = ioctl(timer_fd, TCIOC_SETTIMEOUT, CONFIG_CANOPEN_TIMER_INTERVAL);
	if (ret < 0)
	{
		syslog(LOG_ERR,"ioctl timer timerout err！\n");
		close(timer_fd);
		return ret;
	}


	/* Attach a signal handler to catch the notifications.  NOTE that using
	* signal handler is very slow.  A much more efficient thing to do is to
	* create a separate pthread that waits on sigwaitinfo() for timer events.
	* Much less overhead in that case.
	*/

	act.sa_sigaction = timerDeviceHandler;
	act.sa_flags     = SA_SIGINFO;

	(void)sigfillset(&act.sa_mask);
	(void)sigdelset(&act.sa_mask, 17);//CONFIG_EXAMPLES_TIMER_SIGNO

	ret = sigaction(17, &act, NULL);//CONFIG_EXAMPLES_TIMER_SIGNO
	if (ret != OK)
	{
		syslog(LOG_ERR,"ERROR: Fsigaction failed: %d\n", errno);
		close(timer_fd);
		return ret;
	}

	/* Register a callback for notifications using the configured signal.
	*
	* NOTE: If no callback is attached, the timer stop at the first interrupt.
	*/

	//printf("Attach timer handler\n");

	notify.arg   = NULL;
	notify.pid   = getpid();
	notify.signo = 17;//CONFIG_EXAMPLES_TIMER_SIGNO

	ret = ioctl(timer_fd, TCIOC_NOTIFICATION, (unsigned long)((uintptr_t)&notify));
	if (ret < 0)
	{
		syslog(LOG_ERR,"ERROR: Failed to set the timer handler: %d\n", errno);
		close(timer_fd);
		return ret;
	}


	/* Start the timer */

	//printf("Start the timer\n");

	ret = ioctl(timer_fd,TCIOC_START, 0);
	if (ret < 0)
	{
		syslog(LOG_ERR,"ERROR: Failed to start the timer: %d\n", errno);
		close(timer_fd);
		return ret;
	}
	return OK;
}


/*
*Close canopen timer
*/
/****************************************************************************
 * Name: timerDeviceClose
 *
 * Description:
 *   configure soft timer.
 *
 ****************************************************************************/
int timerDeviceClose(void)
{
	int ret;
	struct sigaction act;

	/* Stop the timer */

	//printf("Stop the timer\n");

	ret = ioctl(timer_fd, TCIOC_STOP, 0);
	if (ret < 0)
	{
		syslog(LOG_ERR,"ERROR: Failed to stop the timer: %d\n", errno);
		close(timer_fd);
		return ret;
	}

	/* Detach the signal handler */

	act.sa_handler = SIG_DFL;
	(void)sigaction(17, &act, NULL);

	/* Show the timer status before starting */


	/* Close the timer driver */
	close(timer_fd);
	syslog(LOG_INFO,"Close the timer driver: %d\n", errno);

	return OK;
}


/******************* (C) COPYRIGHT 2015 Personal Electronics *****END OF FILE****/
