//#include "link.h"
//#include "msg.h"
//#include "platforms/platforms.h"
//#include "main.h"
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
#include "motor_service/motor_serial_msg.h"
#include"motor_service/motor_serial_link.h"
#include <motor_service/elmo_app.h>


extern FPPA_MOTOR_DEVICE Motor_Devive[DEVICE_MAX_MOTOR_NO];
extern int test_rev_msg_from;

//define motor public address
#define MOTOR_DRV_PUBLIC_ID 0

// motor id for 3
#define MOTOR1_DRV_ID 10
#define MOTOR2_DRV_ID 11
#define MOTOR3_DRV_ID 12

link_message_t tx_message = {};
struct msg_info_s motor_info[3] = {};

/**
* @brief motor message  
* @param  link_message_t *msg ���������Ϣ�� 
* @retval uint8_t
*/
uint8_t msg_decode(link_message_t *msg)
{
	switch (msg->msgid)
	{
		case MSG_ID_INFO:		
		// msg_info_s ����������ɵ������
		if((msg->compid >= MOTOR1_DRV_ID) && (msg->compid <= MOTOR3_DRV_ID))
		{
			memcpy((uint8_t *)&motor_info[msg->compid - 10], MSG_PAYLOAD(msg), sizeof(struct msg_info_s));			
		}
		break;	
	}
	test_rev_msg_from = msg->sysid;
	return msg->sysid;
}

/**
* @brief  lvds ���ݷ��ͻص�����
* @param  uint8_t *buff ���ݻ�����ָ��
* @param  uint16_t size �����ֽ���
* @retval uint8_t
*/
uint8_t uart_trans(uint8_t *buff, int size)
{
	//HAL_GPIO_WritePin(GPIOA, LVDS_RE_Pin, GPIO_PIN_SET);
	//UART_BASE::Transmit(LVDS_UART, buff, size, 5000);
	//HAL_GPIO_WritePin(GPIOA, LVDS_RE_Pin, GPIO_PIN_RESET);
	return write(Motor_Devive[0].uart_fd, buff, size);
	//return 1;
}

/**
* @brief ���õ���ŷ�ģʽ
* @param  uin8_t ������id
* @param  uint8_t mode ����ջ�ģʽ��0λ�û���Ĭ�ϣ���1�ٶȻ�
* @retval void
*/
void set_drv_mode(uint8_t id, uint8_t mode)
{
	struct msg_params_mode_s params_mode = { 0 };
	params_mode.run_mode = mode;

	packet_hander_trans(uart_trans, &tx_message, (uint8_t *)&params_mode, id, MSG_ID_PARAMS_MODE, sizeof(struct msg_params_mode_s));
}

/**
* @brief  ����λ�û�����
* @param  uin8_t ������id
* @param  float p �ն�ϵ��
* @param  float d ����ϵ��
* @retval void
*/
void set_posi_params(uint8_t id, float p, float d)
{
	struct msg_params_posi_s msg_params_posi = { 0 };
	msg_params_posi.p_kp = p;
	msg_params_posi.p_kd = d;
	packet_hander_trans(uart_trans, &tx_message, (uint8_t *)&msg_params_posi, id, MSG_ID_PARAMS_POSI, sizeof(struct msg_params_posi_s));
}

/**
* @brief  �����ٶȻ�����
* @param  uin8_t ������id
* @param  float p �ն�ϵ��
* @param  float d ����ϵ��
* @retval void
*/
void set_speed_params(uint8_t id, float p, float d)
{
	struct msg_params_speed_s msg_params_speed = { 0 };
	msg_params_speed.s_kp = p;
	msg_params_speed.s_kd = d;

	packet_hander_trans(uart_trans, &tx_message, (uint8_t *)&msg_params_speed, id, MSG_ID_PARAMS_SPEED, sizeof(struct msg_params_speed_s));
}

/**
* @brief  ��ȡȡ�ò�����Ϣ
* @param  uin8_t ������id
* @retval void
*/
void get_drv_info(uint8_t id)
{
	struct msg_msg_s msg = { 0 };
	msg.id = MSG_ID_INFO;

	packet_hander_trans(uart_trans, &tx_message, (uint8_t *)&msg, id, MSG_ID_CTRL, sizeof(struct msg_msg_s));
}

/**
* @brief  ���������λ��, ʹ�ù�����ַ
* @param  int32_t posi �������λ��
* @retval void
*/
void set_drv_posi(int32_t posi1, int32_t posi2, int32_t posi3)
{
	struct msg_drv_posi_s drv_posi = { 0 };

	drv_posi.posi[0] = posi1;
	drv_posi.posi[1] = posi2;
	drv_posi.posi[2] = posi3;

	packet_hander_trans(uart_trans, &tx_message, (uint8_t *)&drv_posi, MOTOR_DRV_PUBLIC_ID, MSG_ID_DRV_POSI, sizeof(struct msg_drv_posi_s));
}

/**
* @brief  ����������ٶ�, ʹ�ù�����ַ
* @param  int32_t speed ����ٶ�
* @retval void
*/
void set_drv_speed(int32_t speed1, int32_t speed2, int32_t speed3)
{
	struct msg_drv_speed_s drv_speed = { 0 };

	drv_speed.speed[0] = speed1;
	drv_speed.speed[1] = speed2;
	drv_speed.speed[2] = speed3;

	packet_hander_trans(uart_trans, &tx_message, (uint8_t *)&drv_speed, MOTOR_DRV_PUBLIC_ID, MSG_ID_DRV_SPEED, sizeof(struct msg_drv_speed_s));
}

//extern "C" {
void link_parse(uint8_t *ch, int length)
{
	parse_char(msg_decode, ch, length);
}
//}

// ͨѶ����
uint8_t loop = 0;
uint16_t tx_inc = 0;
int drv_com_test(int32_t *val)
{
	loop = (loop + 1) % 7;
	if(loop == 0)
	{
		tx_inc = (tx_inc + 1) % 3;
	}
#if 0
	switch(loop)
	{	
		case 1: set_drv_posi(0, 0, 0);break;
		case 2: set_drv_speed(0, 0, 0);break;
		case 3: set_drv_mode(MOTOR1_DRV_ID + tx_inc, 1);break;
		case 4: set_posi_params(MOTOR1_DRV_ID + tx_inc, 0.06 + tx_inc, 0.15 + tx_inc);break;
		case 5: set_speed_params(MOTOR1_DRV_ID + tx_inc, 0.43 + tx_inc, 0.025 + tx_inc);break;
		case 6: get_drv_info(MOTOR1_DRV_ID + tx_inc);break;		
	}
#else 
	set_drv_posi(val[0] * 100, val[1] * 100, val[2] * 100);
#endif
	return 0;
}

