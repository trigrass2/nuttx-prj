
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <nuttx/lib/math.h>
#include <nuttx/power/motor.h>
 #include "pc_control/pc_control.h"

#include <nuttx/can/can.h>
#include <motor_service/elmo_app.h>
#include "canopen_stack/canfestival.h"

#define CANOPEN_DELAY_L	delay_elmo(2)//usleep(100000)
#define CANOPEN_DELAY_S delay_elmo(1)//usleep(1000)

#define MAX_MOTOR_NO	6

#define POSITION_MODE_ABSOLUTE 		0
#define POSITION_MODE_RELATIVE		1

#define LIMIT_POSITION_LINE				(MOTOR_LINE_ONECIRCLE * 4)

#define ELMO_RUNMODE_SPEED		2
#define ELMO_RUNMODE_POSITION	5



struct Motor_Botton_Strcut
{
	//int Position_Motor_Line;
	int32_t Last_Read_Line;
	int32_t Init_Read_Line;
	float Position_Motor_Angle;//-1440 - 1440
	unsigned char module_id;
	unsigned char run_mode;//2:speed,5:position
	unsigned char reserver[2];
};
struct Motor_Botton_Strcut Position_Elmo[DEVICE_MAX_MOTOR_NO];
unsigned long Test_Time;

//=========================================================================
void Int_To_Buf(signed long data, unsigned char buf[])
{
 	buf[0] = data & 0x00ff;
	data >>=8;
	buf[1] = data & 0x00ff;
	data >>=8;
	buf[2] = data & 0x00ff;
	data >>=8;
	buf[3] = data & 0x00ff;
}

void delay_elmo(unsigned int time)
{
	unsigned int i;
	for (i=0;i<time;i++)
	{
		usleep(10000);
	}
}

UNS8 GetChangeStateResults(UNS8 node_id, UNS8 expected_state, unsigned long timeout_ms)
   {
   unsigned long start_time = 0;
   
   // reset nodes state
   ObjDict_Data.NMTable[node_id] = Unknown_state;

   // request node state
   masterRequestNodeState(&ObjDict_Data, node_id);
   
   //start_time = uptime_ms_proc();
   while(start_time < timeout_ms)
      {
      if (getNodeState(&ObjDict_Data, node_id) == expected_state)
         return 0;
      //sleep_proc(1);
	  CANOPEN_DELAY_S;
			start_time++;
      }
   return 0xFF;
   }

UNS8 ReadSDO(UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS8 dataType, void* data, UNS32* size)
{
	UNS32 abortCode = 0;
	UNS8 res = SDO_UPLOAD_IN_PROGRESS;
	// Read SDO
	UNS8 err = readNetworkDict (&ObjDict_Data, nodeId, index, subIndex, dataType,0);
	if (err)
	{
		return 0xFF;
	}

	err = 0;
	for(;;)
	{
		res = getReadResultNetworkDict (&ObjDict_Data, nodeId, data, size, &abortCode);
		if (res != SDO_UPLOAD_IN_PROGRESS)
		{
			break;   
		}
		CANOPEN_DELAY_S;
		err++;
		if (err > 100)
		{
			break;
		}
		continue;
	}
	closeSDOtransfer(&ObjDict_Data, nodeId, SDO_CLIENT);
	if (res == SDO_FINISHED)
	return 0;
	return 0xFF;   
}

UNS8 SendSDO(UNS8 nodeId, UNS16 index, UNS8 subIndex, UNS32 count, UNS8 dataType, signed long data)
{
   UNS32 abortCode = 0;
   UNS8 res = SDO_UPLOAD_IN_PROGRESS;
   UNS8 canbuf[8];
		UNS8 err = 0;
   // send SDO
   Int_To_Buf(data,canbuf);//
   err = writeNetworkDict(&ObjDict_Data,nodeId,index,subIndex,count,uint32,&canbuf[0],0);
	
	if (err > 0)
	{
		return 0xFF;
	}
	err = 0;
	for(;;)
	{
		res = getReadResultNetworkDict (&ObjDict_Data, nodeId, canbuf, &count, &abortCode);
		if (res != SDO_DOWNLOAD_IN_PROGRESS)
		{
			break;   
		}
		CANOPEN_DELAY_L;
		err++;
		if (err > 100)
		{
			break;
		}
		continue;
	}
	closeSDOtransfer(&ObjDict_Data, nodeId, SDO_CLIENT);
	if (res == SDO_FINISHED)
	{
		return 0;
	}
	return 0xFF;   
}

int Elmo_PDO_Init(unsigned char module_id)
{
	//uint8_t data[10];
	uint8_t err = 0;

	//set SEVER TPDO
	CANOPEN_DELAY_L;
	CANOPEN_DELAY_L;
	if(SendSDO(module_id,0x1600,0x00,4,uint32,0))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1600-00\n",module_id);
	}
	
	CANOPEN_DELAY_L;
	if(SendSDO(module_id,0x1600,0x01,4,uint32,0x607A0020))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1600-01\n",module_id);
	}

	CANOPEN_DELAY_L;

	if(SendSDO(module_id,0x1600,0x02,4,uint32,0x60400010))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1600-02\n",module_id);
		if (err > 2)
		{
			return err;
		}
	}
	CANOPEN_DELAY_L;

	if(SendSDO(module_id,0x1600,0x03,4,uint32,0x60400010))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1600-03\n",module_id);
	}

	CANOPEN_DELAY_L;
	if(SendSDO(module_id,0x1600,0x00,4,uint32,3))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1600-04\n",module_id);
	}

	//SET SEVER RPDO 1,PDO1 comm is default

	if(SendSDO(module_id,0x1A00,0x00,4,uint32,0))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1A00-00\n",module_id);
	}
	
	CANOPEN_DELAY_L;
	if(SendSDO(module_id,0x1A00,0x01,4,uint32,0x60410010))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1A00-01\n",module_id);
	}

	CANOPEN_DELAY_L;

	if(SendSDO(module_id,0x1A00,0x02,4,uint32,0x60630020))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1A00-02\n",module_id);
	}
	CANOPEN_DELAY_L;
	if(SendSDO(module_id,0x1A00,0x00,4,uint32,2))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1A00-002\n",module_id);
	}

	//SET SEVER RPDO 2 map

	if(SendSDO(module_id,0x1A01,0x00,4,uint32,0))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1A01-00\n",module_id);
	}
	
	CANOPEN_DELAY_L;
	if(SendSDO(module_id,0x1A01,0x01,4,uint32,0x60410010))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1A01-01\n",module_id);
	}

	CANOPEN_DELAY_L;

	if(SendSDO(module_id,0x1A01,0x02,4,uint32,0x60630020))//反馈位置信息
	//if(SendSDO(module_id,0x1A01,0x02,4,uint32,0x606C0020))//反馈计算后的速度值
	//if(SendSDO(module_id,0x1A01,0x02,4,uint32,0x60690020))//反馈传感器的速度值
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1A01-02\n",module_id);
	}
	CANOPEN_DELAY_L;
	if(SendSDO(module_id,0x1A01,0x00,4,uint32,2))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1A01-00\n",module_id);
	}

	//SET SEVER RPDO 2 comm
	
	CANOPEN_DELAY_L;

	if(SendSDO(module_id,0x1801,0x03,4,uint32,CONFIG_ELMO_AUTO_BACK_POSITION_TIME*10))//
	{
		err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1801-03\n",module_id);
	}

	CANOPEN_DELAY_L;
	if(SendSDO(module_id,0x1801,0x05,4,uint32,CONFIG_ELMO_AUTO_BACK_POSITION_TIME))//
	{
		//err++;
		syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1801-00\n",module_id);
	}

	/*CANOPEN_DELAY_L;
	if (SendSDO(module_id, 0x2F20, 0x02, 4, uint32, 0X08000000)) //auto return position
	{
		syslog(LOG_WARNING,"TX SDO err:%d 0x2f20-02\n",module_id);
	}*/


	
	CANOPEN_DELAY_L;

	ObjDict_Data.CurrentCommunicationState.csPDO = 1;
	CANOPEN_DELAY_L;
	return err;
}


//module_id: 0:search empty struct,1:search specific module struct
unsigned char Search_Module_Struct_No(unsigned char module_id)

{
	int i;
	for (i = 0; i < APP_MAX_MOTOR_NO; i++)
	{
		if (module_id == Position_Elmo[i].module_id)
		{
			return i;
		}
	}
	return 0XFF;
}


void Elmo_Init(unsigned char module_id,struct can_dev_s *dev)
{

	//CANOpenMasterObject->canHandle = (void *)dev;
	if (dev != NULL)
	{
		ObjDict_Data.canHandle = (void *)dev;
	}
	UNS32 COB_ID_Client_to_Server_Transmit_SDO = 0x600 + module_id;
	UNS32 COB_ID_Server_to_Client_Receive_SDO  = 0x580 + module_id;
	//UNS32 Node_ID_of_the_SDO_Server = module_id;
	UNS32 ExpectedSize = sizeof (UNS32);
	Message temp_pdo;
	uint8_t str_no;
	int32_t	temp;
	UNS32 a;


	str_no = Search_Module_Struct_No(module_id);
	if(str_no > APP_MAX_MOTOR_NO)
	{
		str_no = Search_Module_Struct_No(0);
		if (str_no < APP_MAX_MOTOR_NO)
		{
			Position_Elmo[str_no].module_id  = module_id;
			Position_Elmo[str_no].Position_Motor_Angle = 0;
			Position_Elmo[str_no].Last_Read_Line = 0;
			
		}
		else
		{
			return;
		}

		Position_Elmo[str_no].run_mode = ELMO_RUNMODE_SPEED;
		Position_Elmo[str_no].Last_Read_Line = 0;
		sendSYNCMessage(&ObjDict_Data);
		CANOPEN_DELAY_L;
		CANOPEN_DELAY_L;
		Position_Elmo[str_no].Init_Read_Line = Position_Elmo[str_no].Last_Read_Line;
		Position_Elmo[str_no].run_mode = 0;
		Position_Elmo[str_no].Last_Read_Line = 0;

		if (str_no == 0)//first init
		{
			ObjDict_Data.CurrentCommunicationState.csSDO = 1;
			/* Defining the node Id */
			setNodeId(&ObjDict_Data, 1);

			/* init */
			setState(&ObjDict_Data, Initialisation);
			/*if(Elmo_PDO_Init(module_id) != 0)
			{
				syslog(LOG_WARNING,"elmo:ERROR: elmo canopen init failed: %d\n",module_id);
				Position_Elmo[str_no].module_id  = 0;
				return;
			}
			*/
			CANOPEN_DELAY_L;
			CANOPEN_DELAY_L;
			CANOPEN_DELAY_L;
			CANOPEN_DELAY_L;
		}

		

		if(Position_Elmo[str_no].Init_Read_Line != 0)
		{
			 syslog(LOG_WARNING,"Elmo:elmo mode has inited:%d \n",module_id);
			 //return;
		}
	}

	if (OD_SUCCESSFUL ==  setODentry(&ObjDict_Data, (0x1280 + str_no), 1, &COB_ID_Client_to_Server_Transmit_SDO, &ExpectedSize, RW)
			  && OD_SUCCESSFUL ==  setODentry(&ObjDict_Data, (0x1280 + str_no), 2, &COB_ID_Server_to_Client_Receive_SDO, &ExpectedSize, RW)) 
			 // && OD_SUCCESSFUL ==  setODentry(&ObjDict_Data, (0x1280 + str_no), 3, &Node_ID_of_the_SDO_Server, &ExpectedSize, RW))
	{
		//printf("Init motor %d.....\n",(str_no+1));
	} 
	else
	{
		syslog(LOG_WARNING,"Elmo:ERROR: Object dictionary access failed : %d\n",module_id);
	}


	masterSendNMTstateChange (&ObjDict_Data, module_id, NMT_Enter_PreOperational);

	//insert
		temp_pdo.cob_id = module_id + 0X0700;//0X180;//TPDO1
		temp_pdo.len = 1;
		temp_pdo.rtr = 0;
		temp_pdo.data[0] = 0;
		temp_pdo.data[1] = 0;
		temp_pdo.data[2] = 0;
		temp_pdo.data[3] = 0;
		temp_pdo.data[4] = 0;
		temp_pdo.data[5] = 0;
		temp_pdo.data[6] = 0;
		temp_pdo.data[7] = 0;
		//Canopen_Read_Enable = 1;
		canDeviceMsgSend(&ObjDict_Data,&temp_pdo);
		 //-----------------------------------------------------------------
		 CANOPEN_DELAY_L;

		//insert

	if(Elmo_PDO_Init(module_id) != 0)
	{
		syslog(LOG_WARNING,"Elmo:ERROR: elmo canopen init failed: %d\n",module_id);
		Position_Elmo[str_no].module_id  = 0;
		return;
	}

	CANOPEN_DELAY_L;
		 if (SendSDO(module_id,0x6060,0x00,4,uint32,1))//run mode set
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6060-00\n",module_id);
		 }
		 CANOPEN_DELAY_L;

		 if (SendSDO(module_id,0x6083,0x00,4,uint32,1200000))//acceleration
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6083-00\n",module_id);
		 }
		 CANOPEN_DELAY_L;

		 if (SendSDO(module_id,0x6084,0x00,4,uint32,1200000))//deceleration
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6084-00\n",module_id);
		 }
		 CANOPEN_DELAY_L;

		 if (SendSDO(module_id,0x6085,0x00,4,uint32,1500000))//quick stop deceleration
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6085-00\n",module_id);
		 }

		 CANOPEN_DELAY_L;
		 if (SendSDO(module_id,0x607B,0x01,4,uint32,-LIMIT_POSITION_LINE))//position cycle limit
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x607B-01\n",module_id);
		 }
		 CANOPEN_DELAY_L;
		 if (SendSDO(module_id,0x607B,0x02,4,uint32,LIMIT_POSITION_LINE))//position cycle limit
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x607B-02\n",module_id);
		 }
		 

#if (CONFIG_ELMO_AUTO_BACK_POSITION_TIME > 0)  

			if (SendSDO(module_id, 0x1801, 0x02, 4, uint32, 255)) //set every time return position
			{
				//err++;
				syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1801-02\n",module_id);
			}
#else
			if (SendSDO(module_id, 0x1801, 0x02, 4, uint32, 1)) //set each SYNC return position
			{
				//err++;
				syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1801-02\n",module_id);
			}
#endif 

		CANOPEN_DELAY_L;
		if (SendSDO(module_id,0x1024,0x00,4,uint32,0))//set os read command(RS232 CMD MODE)
		{
			syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
		}
		
		CANOPEN_DELAY_L;
		//RPDO2:set px=0
		temp_pdo.cob_id = module_id + 0X0600;//0X180;//TPDO1
		temp_pdo.len = 8;
		temp_pdo.rtr = 0;
		temp_pdo.data[0] = 0x21;
		temp_pdo.data[1] = 0x23;
		temp_pdo.data[2] = 0x10;
		temp_pdo.data[3] = 1;
		temp_pdo.data[4] = 0;
		temp_pdo.data[5] = 0;
		temp_pdo.data[6] = 0;
		temp_pdo.data[7] = 0;
		Canopen_Read_Enable = 1;
		canDeviceMsgSend(&ObjDict_Data,&temp_pdo);
		CANOPEN_DELAY_L;
		CANOPEN_DELAY_L;

		Canopen_Read_Enable = 0;
		
		//set run mode :"UM=X"
		temp_pdo.cob_id = module_id + 0X0600;//0X180;//TPDO1
		temp_pdo.len = 8;
		temp_pdo.rtr = 0;
		temp_pdo.data[0] = 1;
		temp_pdo.data[1] = 'U';
		temp_pdo.data[2] = 'M';
		temp_pdo.data[3] = '=';
		temp_pdo.data[4] = 2 + '0';
		temp_pdo.data[5] = 0;
		temp_pdo.data[6] = 0;
		temp_pdo.data[7] = 0;
		canDeviceMsgSend(&ObjDict_Data,&temp_pdo);
		CANOPEN_DELAY_L;
		CANOPEN_DELAY_L;
		str_no = Search_Module_Struct_No(module_id);
		Position_Elmo[str_no].run_mode = 2;
		a = 4;
		if (ReadSDO(module_id,0x1023,0x02,uint32,&temp,&a))
		{
			syslog(LOG_WARNING,"Elmo:RX SDO err:%d 0x1023-02\n",module_id);
		}
		else
		{
			if (temp == 0)
			{
				CANOPEN_DELAY_L;
			}
		}


		 CANOPEN_DELAY_L;
		 if (SendSDO(module_id,0x6040,0x00,4,uint32,6))//control word READY SWITCH ON
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
		 }

		 CANOPEN_DELAY_L;
		 if (SendSDO(module_id,0x6040,0x00,4,uint32,7))//control word SWITCH ON
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
		 }
		 CANOPEN_DELAY_L;

		 
		 //-----------------------------------------------------------------
		 //RPDO2:set px=0
		 temp_pdo.cob_id = module_id + 0X0300;//0X180;//TPDO1
		temp_pdo.len = 8;
		temp_pdo.rtr = 0;
		temp_pdo.data[0] = 0X50;
		temp_pdo.data[1] = 0X78;
		temp_pdo.data[2] = 0;
		temp_pdo.data[3] = 0;
		temp_pdo.data[4] = 0;
		temp_pdo.data[5] = 0;
		temp_pdo.data[6] = 0;
		temp_pdo.data[7] = 0;
		Canopen_Read_Enable = 1;
		canDeviceMsgSend(&ObjDict_Data,&temp_pdo);
		 //-----------------------------------------------------------------
		 CANOPEN_DELAY_L;
		 Canopen_Read_Enable = 0;
		 if (SendSDO(module_id,0x6040,0x00,4,uint32,0x0f))//control word
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
		 }


		 CANOPEN_DELAY_L;
		 if (SendSDO(module_id,0x6081,0x00,4,uint32,MOTOR_LINE_ONECIRCLE))//velocity
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6081-00\n",module_id);
		 }

		 masterSendNMTstateChange (&ObjDict_Data, module_id, NMT_Start_Node);//
		
}



int Motor_ELMO_Parameter_Set(unsigned char module_id,int type,void *para)
{
	//int temp_para;
	float temp_fpara;
	unsigned char temp_mode,str_no;
	UNS32 a;
	Message temp_pdo;

	str_no = Search_Module_Struct_No(module_id);
	if (str_no >= APP_MAX_MOTOR_NO)
	{
		syslog(LOG_WARNING,"Elmo:can't find canopen id : %d\n",module_id);
		return 1;
	}

	switch (type)
	{
	case 1://speed
		temp_fpara = *(float *)para;
		if ((temp_fpara > MAX_ELMO_VELOCITY_ANGLE) ||  (temp_fpara < 0))
		{
			return 1;
		}
		if (temp_fpara > 0)
		{
			temp_fpara = (temp_fpara * 10000) / 9; //(para/360) * 25 * 16000
			if (SendSDO(module_id, 0x6081, 0x00, 4, uint32, (long)temp_fpara)) //speed
			{

				syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6081-00\n",module_id);
				return 1;
			}
		}
		else  if(temp_fpara == 0)
		{
			if (SendSDO(module_id, 0x6040, 0x00, 4, uint32, 0x0000)) //quick stop
			{

				syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
				return 1;
			}
		}
		break;
	case 2://acceleration
		temp_fpara = *(float *)para;
		if ((temp_fpara > MAX_ELMO_ACCELERATE_ANGLE) ||  (temp_fpara < 0))
		{
			return 2;
		}
		temp_fpara = (temp_fpara * 10000)/9;//(para/360) * 25 * 16000

		if (SendSDO(module_id,0x6083,0x00,4,uint32,(long)temp_fpara))//acceleration
		{
			syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6083-00\n",module_id);
			return 2;
		}
		break;

	case 3://deceleration
	    temp_fpara = *(float *)para;
		if ((temp_fpara > MAX_ELMO_ACCELERATE_ANGLE) ||  (temp_fpara < 0))
		{
			return 3;
		}
		temp_fpara = (temp_fpara * 10000)/9;//(para/360) * 25 * 16000
		if (SendSDO(module_id,0x6084,0x00,4,uint32,(long)temp_fpara))//deceleration
		 {
			 syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6084-00\n",module_id);
			 return 3;
		 }
		break;
	case 4://mode
		temp_mode = *(unsigned char *)para;
		if (temp_mode == MOTOR_OPMODE_SPEED)
		{
			temp_mode = ELMO_RUNMODE_SPEED;//speed mode
			Position_Elmo[str_no].Init_Read_Line = 0;
		}
		else if ((temp_mode == MOTOR_OPMODE_POSITION_ABSOLUTE) || (temp_mode == MOTOR_OPMODE_POSITION_RELATIVE))
		{
			temp_mode = ELMO_RUNMODE_POSITION; // position mode
		}
		else if (temp_mode == MOTOR_OPMODE_START)//reset mode
		{
			Elmo_Init(module_id,NULL);
			temp_mode = ELMO_RUNMODE_POSITION; // position mode
			return 0;
		} else
		{
			return 4;
		}

		
		
		CANOPEN_DELAY_L;
		if (SendSDO(module_id,0x6040,0x00,4,uint32,6))//control word READY SWITCH ON
		{
			syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
		} 

		//---------------------------------------------------------------

#if (CONFIG_ELMO_AUTO_BACK_POSITION_TIME > 0)  

		if (temp_mode == ELMO_RUNMODE_SPEED)
		{
			CANOPEN_DELAY_L;
			if (SendSDO(module_id, 0x1801, 0x02, 4, uint32, 255)) //set every time return position
			{
				syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1801-02\n",module_id);
			}
		}
		else
		{
			CANOPEN_DELAY_L;
			if (SendSDO(module_id, 0x1801, 0x02, 4, uint32, 1)) //auto return position
			{
				syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x1801-02\n",module_id);
			}
		}

#endif

		CANOPEN_DELAY_L;
		if (SendSDO(module_id,0x1024,0x00,4,uint32,0))//set os read command(RS232 CMD MODE)
		{
			syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
		}
		
		CANOPEN_DELAY_L;
		//RPDO2:set px=0
		temp_pdo.cob_id = module_id + 0X0600;//0X180;//TPDO1
		temp_pdo.len = 8;
		temp_pdo.rtr = 0;
		temp_pdo.data[0] = 0x21;
		temp_pdo.data[1] = 0x23;
		temp_pdo.data[2] = 0x10;
		temp_pdo.data[3] = 1;
		temp_pdo.data[4] = 0;
		temp_pdo.data[5] = 0;
		temp_pdo.data[6] = 0;
		temp_pdo.data[7] = 0;
		Canopen_Read_Enable = 1;
		canDeviceMsgSend(&ObjDict_Data,&temp_pdo);
		CANOPEN_DELAY_L;
		CANOPEN_DELAY_L;

		Canopen_Read_Enable = 0;
		
		//set run mode :"UM=X"
		temp_pdo.cob_id = module_id + 0X0600;//0X180;//TPDO1
		temp_pdo.len = 8;
		temp_pdo.rtr = 0;
		temp_pdo.data[0] = 1;
		temp_pdo.data[1] = 'U';
		temp_pdo.data[2] = 'M';
		temp_pdo.data[3] = '=';
		temp_pdo.data[4] = temp_mode + '0';
		temp_pdo.data[5] = 0;
		temp_pdo.data[6] = 0;
		temp_pdo.data[7] = 0;
		canDeviceMsgSend(&ObjDict_Data,&temp_pdo);
		CANOPEN_DELAY_L;
		CANOPEN_DELAY_L;
		str_no = Search_Module_Struct_No(module_id);
		Position_Elmo[str_no].run_mode = temp_mode;

		a = 4;
		if (ReadSDO(module_id,0x1023,0x02,uint32,&temp_mode,&a))
		{
			syslog(LOG_WARNING,"Elmo:RX SDO err:%d 0x1023-02\n",module_id);
		}
		else
		{
			if (temp_mode == 0)
			{
				CANOPEN_DELAY_L;
			}
		}
		//---------------------------------------------------------------
		

		CANOPEN_DELAY_L;
		if (SendSDO(module_id,0x6040,0x00,4,uint32,0X07))//control word SWITCH  ON
		{
			syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
		}
		CANOPEN_DELAY_L;
		if (SendSDO(module_id,0x6040,0x00,4,uint32,0X0F))//control word MO=1
		{
			syslog(LOG_WARNING,"Elmo:TX SDO err:%d 0x6040-00\n",module_id);
		} 
		CANOPEN_DELAY_L;
		break;
		
	default:
		break;

	}
	return 0;
}
//int Motor_ELMO_State_Get(unsigned char module_id,int type,void *para)
int Motor_ELMO_State_Get(float position,  unsigned char module_id, uint32_t *para)
{
	uint32_t temp;
	unsigned char str_no;

	temp = *para;
	if (temp == 0)
	{
		sendSYNCMessage(&ObjDict_Data);
	}
	else if (temp <= APP_MAX_MOTOR_NO)
	{
		str_no = Search_Module_Struct_No(module_id);
		if (str_no >= APP_MAX_MOTOR_NO)
		{
			syslog(LOG_WARNING,"Elmo:Motor_ELMO_State_Get,can't find canopen id : %d\n",module_id);
			return 1;
		}
		if (Position_Elmo[str_no].Init_Read_Line > 0)
		{
			*para = Position_Elmo[str_no].Init_Read_Line;
		}
		else
		{
			*para = 0xffffffff; 
		}
	}
	
	return 0;
}

float get_angle_360(float angle)
{
	int i;
	if (angle < 0)
	{
		for (i=0;i<5;i++)
		{
			angle += 360;
			if (angle > 0)
			{
				break;
			}
		}
	}
	else if (angle >= 360)
	{
		for (i=0;i<5;i++)
		{
			angle -= 360;
			if (angle < 360)
			{
				break;
			}
		}
	}
	return angle;
}

//=========================================================================
//fun name:			Motor_Position_Move
//position:			motor position target in angle
//module_id:		motor canopen id
//position_mode:	0:absolute,  1: relative
//=========================================================================
int Motor_Position_Move(float position,  unsigned char module_id, int position_mode)
{
	int position_line;
	float position_last,temp_f;
	unsigned char str_no;
	Message temp_pdo;

	str_no = Search_Module_Struct_No(module_id);
	if (str_no >= APP_MAX_MOTOR_NO)
	{
		syslog(LOG_WARNING,"Elmo:can't find canopen id : %d\n",module_id);
		return 1;
	}
	
	//if position mode is absolute ,transition zhe angle to relativve angle 
	if (position_mode == POSITION_MODE_ABSOLUTE)
	{
		position = get_angle_360(position);
		position_last = get_angle_360(Position_Elmo[str_no].Position_Motor_Angle);
		if (position > position_last)
		{
			if ((position -position_last) > 180)
			{
				position = position - 360 -position_last; //negative  through 0 angle
			}
			else
			{
				position -= position_last; //positive
			}
		}
		else
		{
			if ((position_last - position) > 180)
			{
				position = (360 - position_last) + position; //positive
			}
			else
			{
				position = position - position_last ; //negative
			}
		}
	}
	//check angle range
	if ((position > LIMIT_ELMO_POSITION_ANGLE_SOFTWARE) || (position < -LIMIT_ELMO_POSITION_ANGLE_SOFTWARE))
	{
		return 2;
	}

	//check last read position
	if (Position_Elmo[str_no].Last_Read_Line)
	{
		temp_f = Position_Elmo[str_no].Last_Read_Line;
		temp_f = (temp_f/MOTOR_LINE_ONECIRCLE)*360;
		while ((temp_f - Position_Elmo[str_no].Position_Motor_Angle) > 360)
		{
			Position_Elmo[str_no].Position_Motor_Angle += 360;
		}
		while ((temp_f - Position_Elmo[str_no].Position_Motor_Angle) < -360)
		{
			Position_Elmo[str_no].Position_Motor_Angle -= 360;
		}
		Position_Elmo[str_no].Last_Read_Line = 0;
	}


	position_last = Position_Elmo[str_no].Position_Motor_Angle + position;
	//zhe effective angle is -LIMIT_POSITION_ANGLE_HARDWARE to LIMIT_POSITION_ANGLE_HARDWARE,
	if (position_last > LIMIT_POSITION_ANGLE_HARDWARE)
	{
		position_last = position_last - LIMIT_POSITION_ANGLE_HARDWARE - LIMIT_POSITION_ANGLE_HARDWARE;
	}
	else if (position_last < -LIMIT_POSITION_ANGLE_HARDWARE)
	{
		position_last = position_last + LIMIT_POSITION_ANGLE_HARDWARE + LIMIT_POSITION_ANGLE_HARDWARE;
	}
	Position_Elmo[str_no].Position_Motor_Angle = position_last;


	position_last = position_last * (MOTOR_LINE_ONECIRCLE/360);//MOTOR_LINE_ONECIRCLE/360 transition angle to motor line
	position_line  = (int)position_last;

	temp_pdo.cob_id = module_id + 0X0200;//TPDO1
	temp_pdo.len = 8;
	temp_pdo.rtr = 0;
	//position
	temp_pdo.data[0] = position_line & 0xff;
	temp_pdo.data[1] = (position_line >> 8) & 0xff;
	temp_pdo.data[2] = (position_line >> 16) & 0xff;
	temp_pdo.data[3] = (position_line >> 24) & 0xff;
	//control world 
	temp_pdo.data[4] = 0X3F;
	temp_pdo.data[5] = 0;
	//control world 
	temp_pdo.data[6] = 0X0F;
	temp_pdo.data[7] = 0;
	canDeviceMsgSend(&ObjDict_Data,&temp_pdo);

	Position_Elmo[str_no].Init_Read_Line = 0;

	return 0;
}

void CANOpen_Deal_RPDO(unsigned char str[], unsigned int module_id, unsigned char data_long)
{

	int temp_uint;
	int32_t temp;
	int no;
	int32_t buf[5];
	
	temp_uint = module_id & 0x00780;
	module_id &= 0X007F;//can id
	if ((temp_uint == 0x180) || (temp_uint == 0x280))//RPDO1,RPDO2
	{
		no = Search_Module_Struct_No(module_id);
		if (no >= APP_MAX_MOTOR_NO)
		{
			return;
		}
		if ((str[1] == 0x06) || (temp_uint == 0x280))
		{
			if (data_long >= 6)
			{
				temp = str[5];
				temp <<= 8;
				temp += str[4];
				temp <<= 8;
				temp += str[3];
				temp <<= 8;
				temp += str[2];
				buf[0] = 1;
				buf[1] = temp;

				if (str[1] == 0x06)
				{
					Motor_Deal_Back_Data(buf, module_id,ELMO_RETURN_MOVE_OK);
				}
				if (temp_uint == 0x280)
				{
					Motor_Deal_Back_Data(buf, module_id,ELMO_RETURN_TIMER);
				}
				else 
				{
					Motor_Deal_Back_Data(buf, module_id,ELMO_RETURN_READ_OK);
				}
				if (Position_Elmo[no].run_mode == ELMO_RUNMODE_SPEED)
				{
					Position_Elmo[no].Last_Read_Line = temp;
				}
			}
		}
	}
}







