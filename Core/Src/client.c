/*
 * client.c
 *
 *  Created on: Apr 13, 2021
 *      Author: hvunt
 */

#include "main.h"
#include "lwip/tcp.h"
#include "lwip/memp.h"
#include "client.h"
#include "flash_if.h"

static uint32_t Flash_Write_Address;

static struct tcp_pcb *client_pcb = NULL;

static volatile uint8_t is_ready_flag = 0;
static volatile uint8_t connection_status = S_NOT_CONNECTED;

volatile client_t *client = NULL;

static err_t client_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
//static err_t client_sent_callback(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void client_connection_close(struct tcp_pcb *tpcb, client_t *es);
static err_t client_connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t client_send_ack_packet(struct tcp_pcb *tpcb);

static void client_err_callback(void *arg, err_t err);
static err_t client_poll_callback(void *arg, struct tcp_pcb *tpcb);

uint8_t firmware_downloaded(void){
	return is_ready_flag;
}

uint8_t client_is_connected(void){
	return (uint8_t) connection_status;
}

uint8_t client_connect(void) {
	ip_addr_t serverIP;
	client_pcb = tcp_new();

	if (client_pcb != NULL) {
		IP4_ADDR(&serverIP, SERVER_IP_ADDR_0, SERVER_IP_ADDR_1, SERVER_IP_ADDR_2, SERVER_IP_ADDR_3);

		connection_status = S_CONNECTING;

		tcp_err(client_pcb, client_err_callback);
		tcp_poll(client_pcb, client_poll_callback, 10);
		tcp_connect(client_pcb, &serverIP, SERVER_PORT, client_connected_callback);


	} else {
		memp_free(MEMP_TCP_PCB, client_pcb);
		return 1;
	}
	return 0;
}

void client_close_connect(void) {
	client_connection_close(client_pcb, (client_t *) client);
}

static err_t client_connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err) {

	if (err == ERR_OK) {
		client = (client_t*) mem_malloc(sizeof(client_t));

		if (client != NULL) {
			connection_status = S_CONNECTED;
			client->pcb = tpcb;

			FLASH_If_Init();
			FLASH_If_Erase(USER_FLASH_FIRST_PAGE_ADDRESS);
			Flash_Write_Address = USER_FLASH_FIRST_PAGE_ADDRESS;
			client_send_ack_packet(tpcb);

			tcp_arg(tpcb, (client_t*) client);
			tcp_recv(tpcb, client_recv_callback);
		}
	} else {
		client_connection_close(tpcb, (client_t*) client);
	}
	return err;
}

static err_t client_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
	client_t *client = NULL;
	err_t ret_err = err;
	LWIP_ASSERT("arg != NULL", arg != NULL);
	uint32_t data_buffer[CHUNK_SIZE];
	uint16_t count = 0;

	client = (client_t*) arg;
	if (p == NULL) {
		connection_status = S_CLOSING;
		if (client->p_rx == NULL) {
			return ERR_MEM;
		}
		ret_err = ERR_OK;
	} else if (connection_status == S_CONNECTED) {

		if (p->len != p->tot_len){
			return ERR_BUF;
		}
		tcp_recved(tpcb, p->tot_len);
		count = p->len / 4;
		if ((p->len % 4) != 0)
			count++;

		pbuf_copy_partial(p, data_buffer, CHUNK_SIZE, 0);

		FLASH_If_Write(&Flash_Write_Address, data_buffer, count);

		(client->tot_bytes) += p->len;
		client_send_ack_packet(tpcb);

		/*
		 *  if a received buffer is less than the amount of a chunk size,
		 *  so we received a whole firmware
		*/
		if (p->len < CHUNK_SIZE){
			tcp_write(tpcb, client_end_packet, sizeof(client_end_packet), 1);
			client_connection_close(tpcb, client);
			pbuf_free(p);
			is_ready_flag = 1;
		}

		pbuf_free(p);
		ret_err = ERR_OK;
	}

	return ret_err;
}

static void client_err_callback(void *arg, err_t err){
//	if (err == ERR_CONN)
//		connection_status = S_NOT_CONNECTED;
}

static err_t client_poll_callback(void *arg, struct tcp_pcb *tpcb){
	err_t err;
	client_t *client = (client_t *) arg;
	if (client != NULL){
		err = ERR_OK;
	} else {
		/* the device can't reach a firmware server */
		connection_status = S_CONN_ABRT;
		tcp_abort(tpcb);
		err = ERR_ABRT;
	}
	return err;
}

//static err_t client_sent_callback(void *arg, struct tcp_pcb *tpcb, u16_t len) {
//	err_t err;
//
//	return err;
//}

static void client_connection_close(struct tcp_pcb *tpcb, client_t *es) {
	/* remove callbacks */
	tcp_recv(tpcb, NULL);
	tcp_sent(tpcb, NULL);
	tcp_poll(tpcb, NULL, 0);
	tcp_err(tpcb, NULL);

	if (es != NULL) {
		mem_free(es);
	}

	/* close tcp connection */
	tcp_close(tpcb);
}

static err_t client_send_ack_packet(struct tcp_pcb *tpcb) {
	return tcp_write(tpcb, client_ack_packet, sizeof(client_ack_packet), 1);
}
