#pragma once

#include "string.h"
#include "stdint.h"

#define STX	   0x5A //帧头
#define SYS_ID 20   //主站ID

#define NUM_CHECKSUM  2
#define MAX_PAYLOAD_LEN  250

#define MSG_PAYLOAD(msg) ((uint8_t *)(&((msg)->payload[0])))

typedef struct
{
	uint8_t magic;  //帧头
	uint8_t len;    //负载长度, 字节
	uint8_t seq;	//序列码

	uint8_t sysid;  //系统ID
	uint8_t compid; //组件ID
	uint8_t msgid;  //消息ID

	uint8_t payload[MAX_PAYLOAD_LEN + NUM_CHECKSUM]; //载荷
	uint16_t checksum; //校验
	uint8_t ck[2];	//接收校验
}link_message_t;

// 消息包解码
typedef uint8_t(*decode_handle)(link_message_t *msg);

// 数据传输
typedef uint8_t(*trans_handle)(uint8_t *ch, int length);

// 解析
uint8_t parse_char(decode_handle decode, uint8_t *ch, int length);

// message 传输
uint8_t message_hander_trans(trans_handle trans, const link_message_t *msg);

// packet 传输
uint8_t packet_hander_trans(trans_handle trans, link_message_t *msg, const uint8_t *packet, uint8_t compid, uint8_t msgid, uint8_t length);