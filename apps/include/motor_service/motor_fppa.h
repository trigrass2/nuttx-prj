#define CMD_STATE_EMPTY		0
#define CMD_STATE_DO		1
#define CMD_STATE_COMPLETE	2
#define MOTOR_PWM_CMD_START	20

#define PWM_PERCENT_MAX		65535

#define PWM_MAX_FREQUENCY	100000
#define PWM_MIN_FREQUENCY	1500

#define MOTOR_DEFAULT_ZERO	60000

#define MOTOR_UART_BUF_RX_SIZE	100

#define MOTOR_TASK_STATE_EXIT		0
#define MOTOR_TASK_STATE_RUN		1
struct Motor_Ttask_Struct
{
	int 			nush_beacon_cmd;
	Float_Int_type 	nush_beacon_cmd_param;
	int 			task_state;
};
