#pragma once

#include "string.h"
#include "stdint.h"

#define STX	   0x5A //֡ͷ
#define SYS_ID 20   //��վID

#define NUM_CHECKSUM  2
#define MAX_PAYLOAD_LEN  250

#define MSG_PAYLOAD(msg) ((uint8_t *)(&((msg)->payload[0])))

typedef struct
{
	uint8_t magic;  //֡ͷ
	uint8_t len;    //���س���, �ֽ�
	uint8_t seq;	//������

	uint8_t sysid;  //ϵͳID
	uint8_t compid; //���ID
	uint8_t msgid;  //��ϢID

	uint8_t payload[MAX_PAYLOAD_LEN + NUM_CHECKSUM]; //�غ�
	uint16_t checksum; //У��
	uint8_t ck[2];	//����У��
}link_message_t;

// ��Ϣ������
typedef uint8_t(*decode_handle)(link_message_t *msg);

// ���ݴ���
typedef uint8_t(*trans_handle)(uint8_t *ch, int length);

// ����
uint8_t parse_char(decode_handle decode, uint8_t *ch, int length);

// message ����
uint8_t message_hander_trans(trans_handle trans, const link_message_t *msg);

// packet ����
uint8_t packet_hander_trans(trans_handle trans, link_message_t *msg, const uint8_t *packet, uint8_t compid, uint8_t msgid, uint8_t length);