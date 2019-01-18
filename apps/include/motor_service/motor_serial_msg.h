//#pragma once

#include "stdint.h"

/*
	0~9   系统消息包
	10~49  从机返回
	50~100 从机控制
*/

// 系统包
#define MSG_ID_SYSTEM 5
#pragma pack(push,1)
struct msg_system_s
{	
	uint8_t	reboot; // 重启设备
};
#pragma pack(pop) 

// 操作消息包
#define MSG_ID_CTRL 6
#pragma pack(push,1)
struct msg_msg_s
{
	uint8_t	id; // 系统获取设备的返回型消息包
};
#pragma pack(pop) 

// 返回数据包
#define MSG_ID_INFO 10
#pragma pack(push,1)
struct msg_info_s
{
	uint8_t over_curr:1; // 过流
	uint8_t over_volt:1; // 过压

	uint8_t block_rotate:1; //堵转
	uint8_t encoder_lost:1; //编码器丢失

	uint8_t run_mode:4; // 0位置环，1速度环

	int32_t posi; //位置

	int32_t speed; //速度
	
	int32_t posi_ref; //位置参考

	int32_t speed_ref; //速度的参考
	
	// 速度环PID
	float s_kp;
	float s_ki;
	float s_kd;

	//位置环PID
	float p_kp;
	float p_ki;
	float p_kd;
};
#pragma pack(pop) 

// 电机控制模式
#define MSG_ID_PARAMS_MODE 50
#pragma pack(push,1)
struct msg_params_mode_s
{
	uint8_t run_mode; // 0位置环，1速度环
};
#pragma pack(pop) 

// 配置电机位置环参数
#define MSG_ID_PARAMS_POSI 51
#pragma pack(push,1)
struct msg_params_posi_s
{
	float p_kp;
	float p_ki;
	float p_kd;
};
#pragma pack(pop) 

// 配置电机速度环参数
#define MSG_ID_PARAMS_SPEED 52
#pragma pack(push,1)
struct msg_params_speed_s
{
	float s_kp;
	float s_ki;
	float s_kd;
};
#pragma pack(pop) 

// 电机驱动位置控制包， 位置环有效
#define MSG_ID_DRV_POSI 53
#pragma pack(push,1)
struct msg_drv_posi_s
{
	int32_t posi[3];
};
#pragma pack(pop) 

// 电机驱动速度控制包， 速度环有效
#define MSG_ID_DRV_SPEED 54
#pragma pack(push,1)
struct msg_drv_speed_s
{
	int32_t speed[3];
};
#pragma pack(pop) 