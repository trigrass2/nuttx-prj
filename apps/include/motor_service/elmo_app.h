#define MAX_ELMO_ACCELERATE_ANGLE		1800
#define MAX_ELMO_VELOCITY_ANGLE			720
#define LIMIT_ELMO_POSITION_ANGLE_SOFTWARE	1080
#define LIMIT_POSITION_ANGLE_HARDWARE	1440//HARDWARE must big than SOFTWARE,or the direction may be wrong

#define MOTOR_LINE_ONECIRCLE	400000

#define DEVICE_MAX_MOTOR_NO		6//DEVICE MOTOR
#define APP_MAX_MOTOR_NO		CONFIG_MOTOR_USER_NO//USE MOTOR

#define ELMO_RETURN_MOVE_OK		1//position move ok
#define ELMO_RETURN_READ_OK		2//master read data back
#define ELMO_RETURN_TIMER		3//elmo auto return positon data in timer

typedef struct fppa_motor_device
{
  int canopen_fd;							//file fd
  int pwm_fd;
  int uart_fd;								//uart driver device

  unsigned char canopen_id;					//can id
  unsigned char control_type;				//pwm_100,pwm-50,pwm-pulse,da,can,232,
  unsigned char state;
  float 		zero_position;
}FPPA_MOTOR_DEVICE;


void Motor_Deal_Back_Data(int32_t str[], unsigned int module_id,unsigned char data_type);
