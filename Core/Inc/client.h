/*
 * clinet.h
 *
 *  Created on: Apr 13, 2021
 *      Author: hvunt
 */

#ifndef __TCP_CLIENT_H_
#define __TCP_CLIENT_H_

#include "main.h"

#define SERVER_IP_ADDR_0 		192
#define SERVER_IP_ADDR_1 		168
#define SERVER_IP_ADDR_2 		7
#define SERVER_IP_ADDR_3 		60
//#define SERVER_ADDRESS_VPN		"192.168.7.60"

#define SERVER_PORT 58114

#define CHUNK_SIZE 512

enum client_states {
	S_NOT_CONNECTED = 0, S_CONNECTED, S_RECEIVED, S_CLOSING, S_CONNECTING, S_CONN_ABRT
};

/* structure to be passed as argument to the tcp callbacks */
typedef struct client_packet {
	enum client_states state; /* connection status */
	struct tcp_pcb *pcb; /* pointer on the current tcp_pcb */
	struct pbuf *p_rx; /* pointer on pbuf to be received */
	uint32_t tot_bytes;
} client_t;

uint8_t client_connect(void);
void client_close_connect(void);
uint8_t firmware_downloaded(void);
uint8_t client_is_connected(void);

static const char client_ack_packet[] = "ack_packet";
static const char client_end_packet[] = "end_transmision";

#endif //__TCP_CLIENT_H_
