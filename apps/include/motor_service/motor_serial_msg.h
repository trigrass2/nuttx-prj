//#pragma once

#include "stdint.h"

/*
	0~9   ϵͳ��Ϣ��
	10~49  �ӻ�����
	50~100 �ӻ�����
*/

// ϵͳ��
#define MSG_ID_SYSTEM 5
#pragma pack(push,1)
struct msg_system_s
{	
	uint8_t	reboot; // �����豸
};
#pragma pack(pop) 

// ������Ϣ��
#define MSG_ID_CTRL 6
#pragma pack(push,1)
struct msg_msg_s
{
	uint8_t	id; // ϵͳ��ȡ�豸�ķ�������Ϣ��
};
#pragma pack(pop) 

// �������ݰ�
#define MSG_ID_INFO 10
#pragma pack(push,1)
struct msg_info_s
{
	uint8_t over_curr:1; // ����
	uint8_t over_volt:1; // ��ѹ

	uint8_t block_rotate:1; //��ת
	uint8_t encoder_lost:1; //��������ʧ

	uint8_t run_mode:4; // 0λ�û���1�ٶȻ�

	int32_t posi; //λ��

	int32_t speed; //�ٶ�
	
	int32_t posi_ref; //λ�òο�

	int32_t speed_ref; //�ٶȵĲο�
	
	// �ٶȻ�PID
	float s_kp;
	float s_ki;
	float s_kd;

	//λ�û�PID
	float p_kp;
	float p_ki;
	float p_kd;
};
#pragma pack(pop) 

// �������ģʽ
#define MSG_ID_PARAMS_MODE 50
#pragma pack(push,1)
struct msg_params_mode_s
{
	uint8_t run_mode; // 0λ�û���1�ٶȻ�
};
#pragma pack(pop) 

// ���õ��λ�û�����
#define MSG_ID_PARAMS_POSI 51
#pragma pack(push,1)
struct msg_params_posi_s
{
	float p_kp;
	float p_ki;
	float p_kd;
};
#pragma pack(pop) 

// ���õ���ٶȻ�����
#define MSG_ID_PARAMS_SPEED 52
#pragma pack(push,1)
struct msg_params_speed_s
{
	float s_kp;
	float s_ki;
	float s_kd;
};
#pragma pack(pop) 

// �������λ�ÿ��ư��� λ�û���Ч
#define MSG_ID_DRV_POSI 53
#pragma pack(push,1)
struct msg_drv_posi_s
{
	int32_t posi[3];
};
#pragma pack(pop) 

// ��������ٶȿ��ư��� �ٶȻ���Ч
#define MSG_ID_DRV_SPEED 54
#pragma pack(push,1)
struct msg_drv_speed_s
{
	int32_t speed[3];
};
#pragma pack(pop) 