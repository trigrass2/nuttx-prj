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

#define HEADER_LEN  5

typedef enum
{
	FRAMING_INCOMPLETE = 0,
	FRAMING_OK = 1,
	FRAMING_BAD_CRC = 2,
	FRAMING_BAD_SIGNATURE = 3
} framing_t;

typedef enum
{
	PARSE_STATE_UNINIT = 0,
	PARSE_STATE_IDLE,
	PARSE_STATE_GOT_STX,
	PARSE_STATE_GOT_LENGTH,
	PARSE_STATE_GOT_COMPAT_FLAGS,
	PARSE_STATE_GOT_SEQ,
	PARSE_STATE_GOT_SYSID,
	PARSE_STATE_GOT_COMPID,
	PARSE_STATE_GOT_MSGID,
	PARSE_STATE_GOT_PAYLOAD,
	PARSE_STATE_GOT_CRC1,
	PARSE_STATE_GOT_BAD_CRC1,
}parse_state_t;

typedef struct
{
	uint8_t received_sign; //包接收标志
	uint8_t buffer_overrun; // 消息包长度溢出
	uint8_t parse_error; //解析失败计数
	parse_state_t parse_state; //当前解析状态
	uint8_t packet_idx; // 当前包接收计数
	uint8_t current_rx_seq; //接收消息包序列码
	uint8_t current_tx_seq; //发送消息包序列码
}link_status_t;

// 解析临时消息包缓存
link_message_t receive_msg;

// 协议状态
link_status_t link_status;

void change_char_no_int(unsigned char str[])
{
	unsigned char temp;
	temp  = str[0];
	str[0] = str[3];
	str[3] = temp;

	temp  = str[1];
	str[1] = str[2];
	str[2] = temp;
}

// 校验函数
void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
	//将一个字节的数据累加到CRC
	uint8_t tmp;

	tmp = data ^ (uint8_t)(*crcAccum & 0xff);
	tmp ^= (tmp << 4);
	*crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

void crc_init(uint16_t *crcAccum)
{
	*crcAccum = 0xffff;
}

uint16_t crc_calculate(const uint8_t *pBuffer, uint16_t length)
{
	uint16_t crcTmp;
	crc_init(&crcTmp);

	while (length--)
	{
		crc_accumulate(*pBuffer++, &crcTmp);
	}
	return crcTmp;
}

void crc_accumulate_buffer(uint16_t *crcAccum, const uint8_t *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--)
	{
		crc_accumulate(*p++, crcAccum);
	}
}

void start_checksum(link_message_t *msg)
{
	crc_init(&msg->checksum);
}

void update_checksum(link_message_t *msg, uint8_t c)
{
	crc_accumulate(c, &msg->checksum);
}

// 解析错误计数
void parse_error(link_status_t *status)
{
	status->parse_error++;
}

// message 封装
void message_encode(link_message_t *msg, uint8_t compid, uint8_t msgid, uint8_t length)
{
	msg->magic = STX;
	msg->len = length;
	msg->seq = link_status.current_tx_seq;
	msg->sysid = SYS_ID;
	msg->compid = compid;
	msg->msgid = msgid;;

	link_status.current_tx_seq = link_status.current_tx_seq + 1;
	msg->checksum = crc_calculate((uint8_t *)msg + 1, HEADER_LEN);
	crc_accumulate_buffer(&msg->checksum, MSG_PAYLOAD(msg), msg->len);

	*((msg)->len + MSG_PAYLOAD(msg)) = (uint8_t)(msg->checksum & 0xFF);
	*(((msg)->len + 1) + MSG_PAYLOAD(msg)) = (uint8_t)(msg->checksum >> 8);
}

// packet 封装
void packet_encode(link_message_t *msg, const uint8_t *packet, uint8_t compid, uint8_t msgid, uint8_t length)
{
	memcpy(MSG_PAYLOAD(msg), packet, length);

	//memcpy(MSG_PAYLOAD(msg), packet, length);
	//change_char_no_int(&msg->payload[0]);
	//change_char_no_int(&msg->payload[4]);
	//change_char_no_int(&msg->payload[8]);

	/*int i;
	unsigned char *p;
	p = packet;
	for (i=1;i<=12;i++)
	{
		msg->payload[i] = *(p++);
	}
	msg->payload[0] = 0;
	*/
	message_encode(msg, compid, msgid, length);
}

// message 传输
uint8_t message_hander_trans(trans_handle trans, const link_message_t *msg)
{
	return trans((uint8_t*)msg, msg->len + HEADER_LEN + 3);
}

// packet 传输
uint8_t packet_hander_trans(trans_handle trans, link_message_t *msg, const uint8_t *packet, uint8_t compid, uint8_t msgid, uint8_t length)
{
	packet_encode(msg, packet, compid, msgid, length);
	return message_hander_trans(trans, msg);
}

// 解析函数
uint8_t frame_char(link_message_t *rxmsg, link_status_t *status, uint8_t c)
{
	status->received_sign = FRAMING_INCOMPLETE;

	switch (status->parse_state)
	{
	case PARSE_STATE_UNINIT:
	case PARSE_STATE_IDLE:

		if (c == STX)
		{
			status->parse_state = PARSE_STATE_GOT_STX;
			rxmsg->len = 0;
			rxmsg->magic = c;
			start_checksum(rxmsg);
		}
		break;

	case PARSE_STATE_GOT_STX:
		if (c > MAX_PAYLOAD_LEN)
		{
			status->buffer_overrun++;
			parse_error(status);
			status->received_sign = FRAMING_INCOMPLETE;
			status->parse_state = PARSE_STATE_IDLE;
		}
		else
		{
			rxmsg->len = c;
			status->packet_idx = 0;
			update_checksum(rxmsg, c);
			status->parse_state = PARSE_STATE_GOT_LENGTH;
		}
		break;

	case PARSE_STATE_GOT_LENGTH:
		rxmsg->seq = c;
		update_checksum(rxmsg, c);
		status->parse_state = PARSE_STATE_GOT_SEQ;
		break;

	case PARSE_STATE_GOT_SEQ:
		rxmsg->sysid = c;
		update_checksum(rxmsg, c);
		status->parse_state = PARSE_STATE_GOT_SYSID;
		break;

	case PARSE_STATE_GOT_SYSID:
		rxmsg->compid = c;
		if ((c != 0) && (c != SYS_ID))
		{
			status->received_sign = FRAMING_INCOMPLETE;
			status->parse_state = PARSE_STATE_IDLE;
		}
		update_checksum(rxmsg, c);
		status->parse_state = PARSE_STATE_GOT_COMPID;
		break;

	case PARSE_STATE_GOT_COMPID:
		rxmsg->msgid = c;
		update_checksum(rxmsg, c);
		status->parse_state = PARSE_STATE_GOT_MSGID;
		break;

	case PARSE_STATE_GOT_MSGID:
		MSG_PAYLOAD(rxmsg)[status->packet_idx++] = (char)c;
		update_checksum(rxmsg, c);
		if (status->packet_idx == rxmsg->len)
		{
			status->parse_state = PARSE_STATE_GOT_PAYLOAD;
		}
		break;

	case PARSE_STATE_GOT_PAYLOAD:
	{
		if (c != (rxmsg->checksum & 0xFF)) 
		{
			status->parse_state = PARSE_STATE_GOT_BAD_CRC1;
		}
		else 
		{
			status->parse_state = PARSE_STATE_GOT_CRC1;
		}
		rxmsg->ck[0] = c;
		break;
	}

	case PARSE_STATE_GOT_CRC1:
	case PARSE_STATE_GOT_BAD_CRC1:
		if (status->parse_state == PARSE_STATE_GOT_BAD_CRC1 || c != (rxmsg->checksum >> 8)) 
		{
			status->received_sign = FRAMING_BAD_CRC;
		}
		else 
		{
			status->received_sign = FRAMING_OK;
			status->parse_error = 0;
		}
		rxmsg->ck[1] = c;

		status->current_rx_seq = rxmsg->seq;
		status->parse_state = PARSE_STATE_IDLE;
		break;
	case PARSE_STATE_GOT_COMPAT_FLAGS:
		break;
	}
	return status->received_sign;
}

// 解析
uint8_t parse_char(decode_handle decode, uint8_t *ch, int length)
{
	uint8_t msgcnt = 0;
	uint8_t c;
	uint8_t rxsign = FRAMING_BAD_SIGNATURE;

	for (int i = 0; i < length; i++)
	{
		c = *(ch + i);
		rxsign = frame_char(&receive_msg, &link_status, c);

		if (rxsign == FRAMING_BAD_CRC || rxsign == FRAMING_BAD_SIGNATURE)
		{
			parse_error(&link_status);
			link_status.received_sign = FRAMING_INCOMPLETE;
			link_status.parse_state = PARSE_STATE_IDLE;

			if (c == STX)
			{
				link_status.parse_state = PARSE_STATE_GOT_STX;
				receive_msg.len = 0;
				start_checksum(&receive_msg);
			}
		}
		else if (rxsign == FRAMING_OK)
		{
			//memcpy(&parse_msg, &receive_msg, sizeof(link_message_t));
			decode(&receive_msg);
		}
	}
	return msgcnt;
}




