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
#include "canopen_stack/can_device.h"
#include "canopen_stack/canfestival.h"

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <motor_service/elmo_app.h>
#include <syslog.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <nuttx/can/can.h>


/* Configuration ************************************************************/
#define HALF_SECOND_MSEC 500
#define HALF_SECOND_USEC 500000L
#define CAN_PATH "/dev/can3"

extern CO_Data* CANOpenMasterObject;

static int can_fd = 0;
int Canopen_Read_Enable;
void CANOpen_Deal_RPDO(unsigned char str[], unsigned int module_id, unsigned char data_long);

void canDevicePrintErr(unsigned int num, char *buf, unsigned long no)
{
    //printf("%f", num);
    //printf(buf);
    //printf("%f\n", no);
	 caninfo("%d %s %ld \n",num,buf,no);
}

/****************************************************************************
 * Name: canDeviceOpen
 *
 * Description:
 *   open can port.
 *
 ****************************************************************************/
int canDeviceOpen(int type)
{
    type = type;
    can_fd = open(CAN_PATH,O_RDWR );

    if (can_fd < 0){
        syslog(LOG_ERR,"Open Can Device err: %s\n", can_fd);
        return can_fd;
    }
    return OK;

}

/****************************************************************************
 * Name: canDeviceClose
 *
 * Description:
 *   close can port.
 *
 ****************************************************************************/
int canDeviceClose()
{
	int ret = 0;
    ret = close(can_fd);
    if (ret < 0){
        syslog(LOG_ERR,"close can port error: %d\n", ret);
        return can_fd;
    }
    return OK;


}


/****************************************************************************
 * Name: canDeviceMsgSend
 *
 * Description:
 *   sent a can message.
 *
 ****************************************************************************/
unsigned char canDeviceMsgSend(CAN_PORT CANx, Message *m)
{
    struct can_msg_s txmsg;
    int ret,i,nsize = 0;

    txmsg.cm_hdr.ch_id = m->cob_id;
    txmsg.cm_hdr.ch_rtr = m->rtr;
    txmsg.cm_hdr.ch_dlc = m->len;
    for(i=0;i<m->len;i++)
	{
		txmsg.cm_data[i] =	m->data[i];
	}
    nsize = m->len;
    ret = write(can_fd, &txmsg, CAN_MSGLEN(nsize));/////////////////////////////////////////////////////////////////
	if (ret < 1){

            ret = 0;//need deal
			syslog(LOG_WARNING,"can_device:write to can device failt\n");
			return 0XFF;
	}

	return 0;


}


/****************************************************************************
 * Name: canDeviceMsgRead
 *
 * Description:
 *   read a can message.
 *
 ****************************************************************************/
//volatile  static test_read_can_msg;
int canDeviceMsgRead(unsigned char type)
{
    struct can_msg_s rxmsg;
    Message RxMSG;
    int nsize = 0,i;

    type = type;
	//test_read_can_msg = 1;
	//printf("1\n");
	nsize = read(can_fd,&rxmsg,sizeof(struct can_msg_s));
	//test_read_can_msg = 2;
	//printf("2\n");
    if (nsize > 0)
    {

        RxMSG.cob_id = rxmsg.cm_hdr.ch_id;
        RxMSG.rtr = rxmsg.cm_hdr.ch_rtr;
        RxMSG.len = rxmsg.cm_hdr.ch_dlc;
        for(i=0;i<RxMSG.len;i++)
        {
                RxMSG.data[i] = rxmsg.cm_data[i];
        }

		if (Canopen_Read_Enable == 0)
		{
			canDispatch(&ObjDict_Data, &(RxMSG));
            CANOpen_Deal_RPDO(rxmsg.cm_data, rxmsg.cm_hdr.ch_id, rxmsg.cm_hdr.ch_dlc);
		}
        //printf("Nsize:%d  ID:%d  len:%d\n",nsize,RxMSG.cob_id,RxMSG.len);
    }
	return nsize;


}
//
//int canDeviceMsgRead(unsigned char type)
//{
//    struct can_msg_s rxmsg[3];
//    Message RxMSG;
//    int nsize = 0,i;
//
//    type = type;
//	//test_read_can_msg = 1;
//	//printf("1\n");
//	nsize = read(can_fd,rxmsg,sizeof(rxmsg));
//	//test_read_can_msg = 2;
//	//printf("2\n");
//    if (nsize > 0)
//    {
//    	if(nsize > sizeof(struct can_msg_s))
//    	{
//    		syslog(LOG_INFO,"candevice:size is %d\n",nsize);
//    	}
//        RxMSG.cob_id = rxmsg[0].cm_hdr.ch_id;
//        RxMSG.rtr = rxmsg[0].cm_hdr.ch_rtr;
//        RxMSG.len = rxmsg[0].cm_hdr.ch_dlc;
//        for(i=0;i<RxMSG.len;i++)
//        {
//                RxMSG.data[i] = rxmsg[0].cm_data[i];
//        }
//
//		if (Canopen_Read_Enable == 0)
//		{
//			canDispatch(&ObjDict_Data, &(RxMSG));
//            CANOpen_Deal_RPDO(rxmsg[0].cm_data, rxmsg[0].cm_hdr.ch_id, rxmsg[0].cm_hdr.ch_dlc);
//		}
//        //printf("Nsize:%d  ID:%d  len:%d\n",nsize,RxMSG.cob_id,RxMSG.len);
//    }
//	return nsize;
//
//
//}



