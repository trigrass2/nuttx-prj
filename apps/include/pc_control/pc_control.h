typedef union
{
	float fdata;
	uint32_t u_intdata;
	int32_t s_intdata;
  void *point;
	unsigned char str[4];
}Float_Int_type;

typedef struct cmd_struct
{
	Float_Int_type 	params;
	Float_Int_type 	result;
	char 	cmd;
	char 	addr;
	char 	state;
	char 	reserve;
}PC_CMD_STRUCT;

#define PC_CMD_STATE_COMPLETE  2
#define PC_CMD_STATE_EMPTY     0
#define PC_CMD_STATE_CREATE    1  
#define PC_CMD_STATE_ERR       3 
#define PC_CMD_STATE_DO        4 

#define MOTOR_CMD_STATE_COMPLETE  2
#define MOTOR_CMD_STATE_EMPTY     0
#define MOTOR_CMD_STATE_CREATE    1  
#define MOTOR_CMD_STATE_ERR       3 
#define MOTOR_CMD_STATE_DO        4 

#define MOTOR_BACK_CMD_POSITION_OK	100
#define MOTOR_BACK_CMD_TIMER_BACK	101

#define PC_CONTROL_TX_THREAD	1


#define PC_APP_CMD_ERR			0XFF
#define PC_APP_ERR_NO_RETURN	1


#define PC_APP_ADDR_BEACON		10

#define GPS_BEACON_RETURN_FRE	1
#define GPS_BEACON_RETURN_VOL	2

//#define PC_APP_TEST_NET			1

#define DO_CMD_ERR_MIN			1
#define DO_CMD_ERR_MAX			127
#define DO_CMD_ERR_NOERR		0
#define DO_CMD_ERR_ILLICIT		1//illict operate
#define DO_CMD_ERR_FILE			2//file operate err
#define DO_CMD_ERR_FAULT		3//do fault
#define DO_CMD_ERR_CMDNODEFINE	4//zhe cmd is no define
#define DO_CMD_ERR_OUTRANGE		5//zhe cmd params is out of range
#define DO_CMD_ERR_NOANSWER		6//zhe device is no answer

//define zhe mode address
#define PC_ADDR_MOTOR_RX_L1		1
#define PC_ADDR_MOTOR_RX_L2		2
#define PC_ADDR_MOTOR_RX_L3		3
#define PC_ADDR_MOTOR_TX_L1		4
#define PC_ADDR_MOTOR_TX_L2		5
#define PC_ADDR_MOTOR_TX_L3		6
#define PC_ADDR_SYSTEM			8
#define PC_ADDR_BEACON			10
#define PC_ADDR_GPS				11
#define PC_ADDR_ADIS16488		12
#define PC_ADDR_HMC6343			13
#define PC_ADDR_TEMPERATURE		14 
#define PC_ADDR_MODEM			15			
#define PC_ADDR_CURRENT			16
#define PC_ADDR_PID				0XA0
#define PC_ADDR_SATELLLITE		0XA1

#define DATA_LONG_PC_PID	52
#define DATA_LONG_PC_SATE	44
#define DATA_LONG_PC_GPS	8
Float_Int_type	Char_Fint(unsigned char buf[],char data_long);

