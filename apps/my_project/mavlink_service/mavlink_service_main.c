/****************************************************************************
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <syslog.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <debug.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <nuttx/timers/drv_hrt.h>

#include "uORB/uorb/uORB.h"

#include <mavlink_service/mavlink/fppa/mavlink.h>
#include <mavlink_service/mavlink_service.h>
#include <mavlink_service/mav_msg_heartbeat.h>
#include <mavlink_service/mav_msg_imu_data.h>
#include <mavlink_service/mav_msg_raw_imu.h>
#include <mavlink_service/mav_msg_motor_state.h>
#include <mavlink_service/mav_msg_beam_attitude.h>
#include <mavlink_service/mav_msg_beacon_power.h>
#include <mavlink_service/mav_msg_humiture.h>
#include <mavlink_service/mav_msg_rtk_gps.h>

static const struct { const char *message; int (*subscribe)(void *);int (*update)(void *priv, hrt_abstime *);}
mavlink_list[] =
{
		MAV_MSG_HEARTBEAT,\
		MAV_MSG_BEACON_POWER,\
		MAV_MSG_BEAM_ATTITUDE,\
		MAV_MSG_TEMPERATURE,\
		MAV_MSG_ATTITUDE,\
		MAV_MSG_MOTOR_STATE,\
		MAV_MSG_RAW_IMU,\
		MAV_MSG_RTK_GPS,
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MYPORT  5760
#define QUEUE   20
#define BUFFER_SIZE 1024



static int uorb_do_subscribe(void);
static int uorb_do_unsubscribe(void);

/*
 * MAVLINK 功能结构体
 */
static struct mavlink_parse_t g_mavlink =
{
	.initialized 	   = false,										/* 硬件链路初始化标志,fase:未初始化  true:已初始化 */
	.task_pid		   = 0,											/* 任务PID号 */
	.should_exit	   = true,										/* 退出任务开关,false:保持任务循环  true:退出任务循环 */
	.subscribe	 	   = false,										/* uORB订阅标志,false:未订阅所有话题 true:已订阅所有话题 */
	.sockfd 		   = 0,											/* socket 文件描述符 */
	.client_fd	 	   = 0,											/* UDP客户端 文件描述符 */
};

 
 /****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: send_message_ethernet
 *
 * Description:
 *   send mavlink message
 *
 * Input Parameters:
 *   pointer  - pointer of message.
 *
 * Returned Value:
 *   the size of message.
 *
 ****************************************************************************/
inline int send_message_ethernet(void *priv,void *buff,int len,int flog)
{
	struct mavlink_parse_t *mav = priv;
	int ret = 0;
#ifdef CONFIG_APP_MAVLINK_SERVICE_NET_PROTOCOL_TCP

	ret = send(mav->client_fd, buff, len, flog);
	if(ret < 0 ){
		syslog(LOG_ERR, "mavlink: send error:%d\n",ret);
	}

#endif

#ifdef CONFIG_APP_MAVLINK_SERVICE_NET_PROTOCOL_UDP

	ret = sendto(mav->sockfd,buff,len,0,(struct sockaddr *)&(mav->udp_client),sizeof(mav->udp_client));
	if(ret < 0 ){
		syslog(LOG_ERR, "mavlink: send error:%d\n",ret);
	}


#endif

	return ret;

}

/****************************************************************************
 * Name: close_message_ethernet
 *
 * Description:
 *   close mavlink message
 *
 * Input Parameters:
 *   pointer  - pointer of message.
 *
 * Returned Value:
 *   the size of message.
 *
 ****************************************************************************/
int close_message_ethernet(void *priv)
{
	struct mavlink_parse_t *mav = priv;
	int ret = 0;

	/*
	 * unsubscribe all orb message before close net port
	 */
	ret = uorb_do_unsubscribe();
	if(ret < 0 ){
		syslog(LOG_ERR, "[MAV]: unsubscribe orb message error:%d\n",ret);
		return ret;
	}else{
		syslog(LOG_INFO, "[MAV]: unsubscribed all orb message %d\n",ret);
	}

#ifdef CONFIG_APP_MAVLINK_SERVICE_NET_PROTOCOL_TCP

	ret = close(mav->client_fd);
	if(ret < 0 ){
		syslog(LOG_ERR, "[MAV]: TCP port  close error:%d\n",ret);
	}else{
		syslog(LOG_INFO, "[MAV]: TCP port closed %d\n",ret);
	}

#endif

#ifdef CONFIG_APP_MAVLINK_SERVICE_NET_PROTOCOL_UDP
	ret = close(mav->client_fd);
	if(ret < 0 ){
		syslog(LOG_ERR, "[MAV]: UDP port close error:%d\n",ret);
	}else{
		syslog(LOG_INFO, "[MAV]: UDP port closed %d\n",ret);
	}

#endif
	return ret;

}

/****************************************************************************
 * Name: uorb_do_subscribe
 *
 * Description:
 *   subscribe uorb message.
 *
 * Input Parameters:
 *
 * Returned Value:.
 *
 ****************************************************************************/
static int uorb_do_subscribe(void)
{
	int ret = -1;
	if (!g_mavlink.subscribe){

       /*
		* subscribe message from orb
		*/
		for(int i = 0; i < sizeof(mavlink_list)/sizeof(mavlink_list[0]); i++ ){

			ret = mavlink_list[i].subscribe(&g_mavlink);

			if(ret < 0){
				syslog(LOG_ERR, "[MAV]: subscribe %s error:%d\n",mavlink_list[i].message,ret);
			}
		}

		g_mavlink.subscribe = true;

	}

	return 0;
}

/****************************************************************************
 * Name: uorb_do_unsubscribe
 *
 * Description:
 *   subscribe uorb message.
 *
 * Input Parameters:
 *
 * Returned Value:.
 *
 ****************************************************************************/
static int uorb_do_unsubscribe(void)
{
//	int ret = 0;
	if (g_mavlink.subscribe){

		//unsubscribe message from adis16488

//		ret = orb_unsubscribe(g_mavlink.fd[IMU_RAW]);
//
//		if(ret < 0 ){
//			syslog(LOG_ERR, "[MAV] mavlink:faile to unsubscribe adis16488_adis16488！:%d\n",ret);
//			return ret;
//		}else{
//			syslog(LOG_INFO, "[MAV] mavlink:unsubscribed adis16488_adis16488 %d\n",ret);
//
//			memset(&adis16488_adis16488, 0, sizeof(adis16488_adis16488));
//		}
//
//		//subscribe message from attitude
//
//		ret = orb_unsubscribe(g_mavlink.fd[ATTITUDE]);
//
//		if(ret < 0 ){
//			syslog(LOG_ERR, "[MAV] mavlink:faile to unsubscribe imu_data！:%d\n",ret);
//			return ret;
//		}else{
//			syslog(LOG_INFO, "[MAV] mavlink:unsubscribed imu_data %d\n",ret);
//
//			memset(&(g_mavlink.msg_attitude->data), 0, sizeof((g_mavlink.msg_attitude->data)));
//		}

		g_mavlink.subscribe = false;

		return OK;
	}
	return OK;
}

/****************************************************************************
 * Name: update_and_upload_message
 *
 * Description:
 *   update and upload_message .
 *
 * Input Parameters:
 *
 * Returned Value:.
 *
 ****************************************************************************/
static int update_and_upload_message(hrt_abstime *curr)
{

	for(int i = 0; i < sizeof(mavlink_list)/sizeof(mavlink_list[0]); i++ ){
		mavlink_list[i].update(&g_mavlink,curr);
	}

	return 0;
}

/****************************************************************************
 * Name: mavlink_packet_parse_task
 *
 * Description:
 *   parse mavlink message.
 *
 * Input Parameters:
 *
 * Returned Value:.
 *
 ****************************************************************************/

int mavlink_packet_parse_task(int argc, FAR char *argv[])
{
	while(!g_mavlink.should_exit)
	{

#ifdef CONFIG_APP_MAVLINK_SERVICE_NET_PROTOCOL_TCP
		if(!g_mavlink.initialized){
			// TCP socket
			g_mavlink.sockfd = socket(AF_INET,SOCK_STREAM,0);
			if (g_mavlink.sockfd<0)
			{
				syslog(LOG_ERR, "mavlink: creat socket error:%d\n",g_mavlink.sockfd);
				 exit(1);
			}

			//clean
			memset(&(g_mavlink.server_addr),0,sizeof(g_mavlink.server_addr));
			g_mavlink.server_addr.sin_family 		= AF_INET;
			g_mavlink.server_addr.sin_addr.s_addr 	= htonl(INADDR_ANY);
			g_mavlink.server_addr.sin_port 			= htons(5760);

			//bind the socket
			int ret = bind(g_mavlink.sockfd,(struct sockaddr *)(&(g_mavlink.server_addr)),sizeof(g_mavlink.server_addr));
			if (ret<0){
				syslog(LOG_ERR, "mavlink: socket bind error:%d\n",ret);
				return ret;
			}

			//listen the port
			ret = listen(g_mavlink.sockfd,2);
			if (ret < 0){
				 syslog(LOG_ERR, "mavlink: socket listen err:%d\n",ret);
			}

			//accept client
			g_mavlink.client_fd = accept(g_mavlink.sockfd,NULL,NULL);
			if (g_mavlink.client_fd < 0){
				syslog(LOG_ERR, "mavlink: socket accept error %d\n",g_mavlink.client_fd);
				return g_mavlink.client_fd;
			}

			g_mavlink.initialized = true;
		}
#endif

#ifdef CONFIG_APP_MAVLINK_SERVICE_NET_PROTOCOL_UDP
		if(!g_mavlink.initialized){
			// UDP socket
			g_mavlink.sockfd = socket(AF_INET,SOCK_DGRAM,0);
			if (g_mavlink.sockfd<0)
			{
				syslog(LOG_ERR, "mavlink: creat socket error:%d\n",g_mavlink.sockfd);
				 exit(1);
			}

			g_mavlink.udp_client.sin_family = AF_INET;
			g_mavlink.udp_client.sin_addr.s_addr = inet_addr(argv[3]);
			g_mavlink.udp_client.sin_port = htons(14550);

			g_mavlink.initialized = true;
		}
#endif
		/*
		 * send mavlink parket
		 */
		else{

			/* subscribe uorb message */
			uorb_do_subscribe();

			hrt_abstime abscurr = hrt_absolute_time();

			/* poll all subscribed uorb message */
			int poll_ret = poll(g_mavlink.fds, 1, 500);

			/* handle the poll result */
			if (poll_ret < 0){

				/* use a counter to prevent flooding (and slowing us down) */
				syslog(LOG_ERR, "mavlink: ERROR return value from poll(): %d\n", poll_ret);

			}else{

				update_and_upload_message(&abscurr);
			}
		}
	}
	return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name: usage
 *
 * Description:
 *   application use infomations.
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   none.
 *
 ****************************************************************************/
static void usage(void)
{
	printf(" mavlink_service_main application\n");
	printf(" <cmd> <start> <ip address> start the program.\n");
	printf(" <cmd> <stop> stop the program\n");
	printf(" <cmd> <status> print the program status.\n");
	printf("\n");
}

/****************************************************************************
 * mavlink_service_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int mavlink_service_main(int argc, char *argv[])
#endif
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start the application.
	 */
	if (!strcmp(argv[1], "start")) {

		if(g_mavlink.should_exit == true){

			if(argc != 3){
				usage();
				return -EINVAL;
			}

			g_mavlink.task_pid = task_create(CONFIG_APP_MAVLINK_SERVICE_PROGNAME, \
											 CONFIG_APP_MAVLINK_SERVICE_PRIORITY, \
											 CONFIG_APP_MAVLINK_SERVICE_STACKSIZE,\
											 mavlink_packet_parse_task,\
											 argv);

			if (g_mavlink.task_pid < 0){

				syslog(LOG_ERR, "[MAV] ERROR: Failed to start mavlink task: %d\n",g_mavlink.task_pid);

				printf("[MAV] ERROR: Failed to start mavlink task: %d\n",g_mavlink.task_pid);

				return g_mavlink.task_pid;
			}

			g_mavlink.should_exit = false;

			return OK;

		}else{

			syslog(LOG_WARNING, "[MAV] WARN: mavlink task already run\n");

			printf("[MAV] WARN: mavlink task already run\n");

			return OK;
		}
	}
	/*
	 * Stop the application.
	 */
	else if (!strcmp(argv[1], "stop")) {

		int ret = close_message_ethernet(&g_mavlink);

		if(ret < 0 ){

			syslog(LOG_ERR, "[MAV]mavlink: close error:%d\n",ret);

			printf("[MAV]mavlink: close error:%d\n",ret);

		}else{
			syslog(LOG_INFO, "[MAV]mavlink: closed %d\n",ret);

			g_mavlink.should_exit = true;

			g_mavlink.initialized = false;
		}

		return ret;
	}

	/*
	 * print the application status.
	 */
	else if (!strcmp(argv[1], "status")) {

		return OK;
	}

	/*
	 * Set parameters of the application.
	 */
	else if (!strcmp(argv[1], "-s")) {

		if(!strcmp(argv[2], "ip")){
#ifdef CONFIG_APP_MAVLINK_SERVICE_NET_PROTOCOL_UDP
		g_mavlink.udp_client.sin_addr.s_addr = inet_addr(argv[3]);
#endif
		}
		else{
			usage();
		}
		return OK;
	}

	/*
	 * Get parameters of the application.
	 */
	else if (!strcmp(argv[1], "-r")) {

		return OK;
	}

	/*
	 * Print help information.
	 */
	else{

		usage();

		return -EINVAL;
	}
}


