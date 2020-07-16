/***************************************************************************//**
 *   @file   wifi.c
 *   @brief  Wifi implementation for ESP8266
 *   @author Mihail Chindris (mihail.chindris@analog.com)
********************************************************************************
 *   @copyright
 * Copyright 2020(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdlib.h>
#include "wifi.h"
#include "at_parser.h"
#include "error.h"
#include "util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define INVALID_ID	0xffffffff

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Structure storing data used by a socket */
struct socket_desc {
	/* Buffer given to at_parser */
	struct circular_buffer	*cb;
	/* Circular buffer size */
	uint32_t		cb_size;
	/* Socket type */
	enum socket_protocol	type;
	/* Connection id */
	uint32_t		conn_id;
	/* States of a socket structure */
	enum {
		/* The socket structure is unused */
		SOCKET_UNUSED,
		/* Socket structure have been initialized */
		SOCKET_DISCONNECTED,
		/* Socket connected to a server or to a client */
		SOCKET_CONNECTED
	}			state;
};

/* Wifi descriptor */
struct wifi_desc {
	/* Sockets */
	struct socket_desc		sockets[MAX_CONNECTIONS];
	/* Reference to the AT parser */
	struct at_desc			*at;
	/* Network interface */
	struct network_interface	interface;
	/* Will be used in callback */
	int32_t				conn_id_to_sock_id[MAX_CONNECTIONS];
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/


static int32_t wifi_socket_open(struct wifi_desc *desc, uint32_t *sock_id,
				enum socket_protocol proto, uint32_t buff_size);
static int32_t wifi_socket_close(struct wifi_desc *desc, uint32_t sock_id);
static int32_t wifi_socket_connect(struct wifi_desc *desc, uint32_t sock_id,
				   struct socket_address *addr);
static int32_t wifi_socket_disconnect(struct wifi_desc *desc, uint32_t sock_id);
static int32_t wifi_socket_send(struct wifi_desc *desc, uint32_t sock_id,
				const void *data, uint32_t size);
static int32_t wifi_socket_recv(struct wifi_desc *desc, uint32_t sock_id,
				void *data, uint32_t size);
static int32_t wifi_socket_sendto(struct wifi_desc *desc, uint32_t sock_id,
				  const void *data, uint32_t size,
				  struct socket_address to);
static int32_t wifi_socket_recvfrom(struct wifi_desc *desc, uint32_t sock_id,
				    void *data, uint32_t size,
				    struct socket_address *from);

/* Returns the index of a socket in SOCKET_UNUSED state */
static inline int32_t _wifi_get_unused_socket(struct wifi_desc *desc,
		uint32_t *idx)
{
	uint32_t i;

	for (i = 0; i < MAX_CONNECTIONS; i++)
		if (desc->sockets[i].state == SOCKET_UNUSED) {
			desc->sockets[i].state = SOCKET_DISCONNECTED;
			*idx = i;

			return SUCCESS;
		}

	/* All the available connections are used */
	return -EMLINK;
}

/* Marks the socket at the index id as SOCKET_UNUSED */
static inline void _wifi_release_socket(struct wifi_desc *desc, uint32_t id)
{
	desc->sockets[id].state = SOCKET_UNUSED;
}

/* Set the socket connection ID with an ID that is not used yet */
static inline uint32_t _wifi_get_unused_conn(struct wifi_desc *desc,
		uint32_t sock_id)
{
	uint32_t i;

	for (i = 0; i < MAX_CONNECTIONS; i++)
		if (desc->conn_id_to_sock_id[i] == INVALID_ID) {
			desc->conn_id_to_sock_id[i] = sock_id;
			desc->sockets[sock_id].conn_id = i;

			return SUCCESS;
		}

	return -EMLINK;
}

/* Unset the socket connection ID and mark it as available */
static inline void _wifi_release_conn(struct wifi_desc *desc,
				      uint32_t sock_id)
{
	uint32_t conn_id;

	conn_id = desc->sockets[sock_id].conn_id;
	if (conn_id != INVALID_ID) {
		/*
		 * Connection can be released both from callback or
		 * stop_connection command, and it shouldn't be released twice
		 */
		desc->sockets[sock_id].conn_id = INVALID_ID;
		desc->conn_id_to_sock_id[conn_id] = INVALID_ID;
	}
}

/* Connect internal functions to the network interface */
static void wifi_init_interface(struct wifi_desc *desc)
{
	desc->interface.net = desc;
	desc->interface.socket_open =
		(int32_t (*)(void *, uint32_t *, enum socket_protocol,
			     uint32_t))
		wifi_socket_open;
	desc->interface.socket_close =
		(int32_t (*)(void *, uint32_t))
		wifi_socket_close;
	desc->interface.socket_connect =
		(int32_t (*)(void *, uint32_t, struct socket_address *))
		wifi_socket_connect;
	desc->interface.socket_disconnect =
		(int32_t (*)(void *, uint32_t))
		wifi_socket_disconnect;
	desc->interface.socket_send =
		(int32_t (*)(void *, uint32_t, const void *, uint32_t))
		wifi_socket_send;
	desc->interface.socket_recv =
		(int32_t (*)(void *, uint32_t, void *, uint32_t))
		wifi_socket_recv;
	desc->interface.socket_sendto =
		(int32_t (*)(void *, uint32_t, const void *, uint32_t,
			     const struct socket_address *))
		wifi_socket_sendto;
	desc->interface.socket_recvfrom =
		(int32_t (*)(void *, uint32_t, void *, uint32_t,
			     struct socket_address *))
		wifi_socket_recvfrom;
}

/* Callback to be submmited to the at_parser to get notification when a
connection is created or closed */
static void _wifi_connection_callback(void *ctx, enum at_event event,
				      uint32_t conn_id,
				      struct circular_buffer **cb)
{
	struct wifi_desc	*desc = ctx;
	int32_t			sock_id;

	sock_id = desc->conn_id_to_sock_id[conn_id];
	if (event == AT_NEW_CONNECTION) {
		if (sock_id != INVALID_ID)
			*cb = desc->sockets[sock_id].cb;
		else
			*cb = NULL;
	} else if (event == AT_CLOSED_CONNECTION) {
		if (sock_id != INVALID_ID) {
			desc->sockets[sock_id].state = SOCKET_DISCONNECTED;
			_wifi_release_conn(desc, sock_id);
		}
	}
}

/**
 * @brief Allocate resources and initializes a wifi descriptor
 * @param desc - Address where to store the wifi descriptor
 * @param param - Initializing data
 * @return
 *  - \ref SUCCESS : On success
 *  - \ref FAILURE : Otherwise
 */
int32_t wifi_init(struct wifi_desc **desc, struct wifi_init_param *param)
{
	struct wifi_desc	*ldesc;
	struct at_init_param	at_param;
	int32_t			result;
	union in_out_param	par;

	if (!desc || !param)
		return FAILURE;

	ldesc = (struct wifi_desc *)calloc(1, sizeof(*ldesc));
	if (!ldesc)
		return FAILURE;

	memset(ldesc->conn_id_to_sock_id, (int8_t)INVALID_ID,
	       sizeof(ldesc->conn_id_to_sock_id));

	at_param.irq_desc = param->irq_desc;
	at_param.uart_desc = param->uart_desc;
	at_param.uart_irq_conf = param->uart_irq_conf;
	at_param.uart_irq_id = param->uart_irq_id;
	at_param.connection_callback = _wifi_connection_callback;
	at_param.callback_ctx = ldesc;

	result = at_init(&ldesc->at, &at_param);
	if (IS_ERR_VALUE(result))
		goto ldesc_err;

	wifi_init_interface(ldesc);

	result = at_run_cmd(ldesc->at, AT_RESET, AT_EXECUTE_OP, NULL);
	if (IS_ERR_VALUE(result))
		goto at_err;

	par.in.wifi_mode = CLIENT;
	result = at_run_cmd(ldesc->at, AT_SET_OPERATION_MODE, AT_SET_OP, &par);
	if (IS_ERR_VALUE(result))
		goto at_err;

	par.in.conn_type = MULTIPLE_CONNECTION;
	result = at_run_cmd(ldesc->at, AT_SET_CONNECTION_TYPE, AT_SET_OP, &par);
	if (IS_ERR_VALUE(result))
		goto at_err;
	*desc = ldesc;

	return SUCCESS;
at_err:
	at_remove(ldesc->at);
ldesc_err:
	free(ldesc);

	return FAILURE;
}

/**
 * @brief Deallocate resources from the wifi descriptor
 * @param desc - Wifi descriptor
 * @return
 *  - \ref SUCCESS : On success
 *  - \ref FAILURE : Otherwise
 */
int32_t wifi_remove(struct wifi_desc *desc)
{
	uint32_t i;

	if (!desc)
		return FAILURE;

	for (i = 0; i < MAX_CONNECTIONS; i++)
		wifi_socket_close(desc, i);

	wifi_disconnect(desc);

	at_remove(desc->at);
	free(desc);

	return SUCCESS;
}

/**
 * @brief Connect to a wifi network
 * @param desc - Socket descriptor
 * @param ssid - Wifi SSID
 * @param pass - Wifi password
 * @return
 *  - \ref SUCCESS : On success
 *  - \ref FAILURE : Otherwise
 */
int32_t wifi_connect(struct wifi_desc *desc, const char *ssid,
		     const char *pass)
{
	union in_out_param	param;

	if (!desc)
		return FAILURE;

	str_to_at(&param.in.network.ssid, (uint8_t *)ssid);
	str_to_at(&param.in.network.pwd, (uint8_t *)pass);
	return at_run_cmd(desc->at, AT_CONNECT_NETWORK, AT_SET_OP, &param);
}

/**
 * @brief Disconnect from the wifi network
 * @param desc - Socket descriptor
 * @return
 *  - \ref SUCCESS : On success
 *  - \ref FAILURE : Otherwise
 */
int32_t wifi_disconnect(struct wifi_desc *desc)
{
	if (!desc)
		return FAILURE;

	return at_run_cmd(desc->at, AT_DISCONNECT_NETWORK, AT_EXECUTE_OP, NULL);
}

/**
 * @brief Get network interface reference
 * @param desc - Socket descriptor
 * @param net - Address where to store the reference to the network interface
 * @return
 *  - \ref SUCCESS : On success
 *  - \ref FAILURE : Otherwise
 */
int32_t wifi_get_network_interface(struct wifi_desc *desc,
				   struct network_interface **net)
{
	if (!desc || !net)
		return FAILURE;

	*net = &desc->interface;

	return SUCCESS;
}

/**
 * @brief Get ip
 * @param desc - Wifi descriptor
 * @param ip_buff - Buffer where to copy the null terminated ip string
 * @param buff_size - Size of the buffer
 * @return
 *  - \ref SUCCESS : On success
 *  - \ref -EINVAL : For invalid parameters
 *  - \ref -ENOMEM : If buffer is too small
 */
int32_t wifi_get_ip(struct wifi_desc *desc, char *ip_buff, uint32_t buff_size)
{
	int32_t			ret;
	union in_out_param	result;
	uint8_t			*buff;

	if (!desc || !ip_buff)
		return -EINVAL;

	ret = at_run_cmd(desc->at, AT_GET_IP, AT_EXECUTE_OP, &result);
	if (IS_ERR_VALUE(ret))
		return ret;

	at_to_str(&buff, &result.out.result);
	if (strlen((char *)buff) + 1 > buff_size)
		return (-ENOMEM);

	strcpy(ip_buff, (char *)buff);

	return SUCCESS;
}

/** @brief See \ref network_interface.socket_open */
static int32_t wifi_socket_open(struct wifi_desc *desc, uint32_t *sock_id,
				enum socket_protocol proto, uint32_t buff_size)
{
	uint32_t	id;
	int32_t		ret;

	if (!desc || !sock_id)
		return FAILURE;

	ret = _wifi_get_unused_socket(desc, &id);
	if (IS_ERR_VALUE(ret))
		return ret;

	ret = cb_init(&desc->sockets[id].cb, buff_size);
	if (IS_ERR_VALUE(ret)) {
		_wifi_release_socket(desc, id);
		return ret;
	}

	desc->sockets[id].type = proto;
	desc->sockets[id].cb_size = buff_size;

	*sock_id = id;

	return SUCCESS;
}

/** @brief See \ref network_interface.socket_close */
static int32_t wifi_socket_close(struct wifi_desc *desc, uint32_t sock_id)
{
	struct socket_desc	*sock;
	int32_t			ret;

	if (!desc || sock_id >= MAX_CONNECTIONS)
		return FAILURE;

	sock = &desc->sockets[sock_id];
	if (sock->state == SOCKET_UNUSED)
		return SUCCESS;

	ret = wifi_socket_disconnect(desc, sock_id);
	if (IS_ERR_VALUE(ret))
		return ret;

	_wifi_release_socket(desc, sock_id);
	cb_remove(sock->cb);
	sock->cb = NULL;

	return SUCCESS;
}

/** @brief See \ref network_interface.socket_connect */
static int32_t wifi_socket_connect(struct wifi_desc *desc, uint32_t sock_id,
				   struct socket_address *addr)
{
	union in_out_param	param;
	uint32_t		ret;
	struct socket_desc	*sock;

	if (!desc || !addr || sock_id >= MAX_CONNECTIONS)
		return FAILURE;

	sock = &desc->sockets[sock_id];
	if (sock->state == SOCKET_UNUSED)
		return -ENOENT;

	if (sock->state == SOCKET_CONNECTED)
		return -EISCONN;

	ret = _wifi_get_unused_conn(desc, sock_id);
	if (IS_ERR_VALUE(ret))
		return ret;

	str_to_at(&param.in.connection.addr, (uint8_t *)addr->addr);
	param.in.connection.port = addr->port;
	param.in.connection.id = sock->conn_id;
	param.in.connection.soket_type = sock->type;

	ret = at_run_cmd(desc->at, AT_START_CONNECTION, AT_SET_OP, &param);
	if (IS_ERR_VALUE(ret)) {
		_wifi_release_conn(desc, sock_id);
		return ret;
	}

	sock->state = SOCKET_CONNECTED;

	return SUCCESS;
}

/** @brief See \ref network_interface.socket_disconnect */
static int32_t wifi_socket_disconnect(struct wifi_desc *desc, uint32_t sock_id)
{
	union in_out_param	param;
	uint32_t		ret;
	struct socket_desc	*sock;

	if (!desc || sock_id >= MAX_CONNECTIONS)
		return FAILURE;

	sock = &desc->sockets[sock_id];
	if (sock->state == SOCKET_UNUSED)
		return -ENOENT;

	if (sock->state == SOCKET_DISCONNECTED)
		/* A socket can be disconnected by the peer */
		return SUCCESS;

	param.in.conn_id = sock->conn_id;
	ret = at_run_cmd(desc->at, AT_STOP_CONNECTION, AT_SET_OP,
			 &param);
	if (IS_ERR_VALUE(ret))
		return ret;
	_wifi_release_conn(desc, sock_id);

	sock->state = SOCKET_DISCONNECTED;

	return SUCCESS;
}

/** @brief See \ref network_interface.socket_send */
static int32_t wifi_socket_send(struct wifi_desc *desc, uint32_t sock_id,
				const void *data, uint32_t size)
{
	union in_out_param	param;
	uint32_t		ret;
	struct socket_desc	*sock;
	uint32_t		to_send;
	uint32_t		i;

	if (!desc || sock_id >= MAX_CONNECTIONS)
		return -EINVAL;

	sock = &desc->sockets[sock_id];
	if (sock->state != SOCKET_CONNECTED)
		return -ENOTCONN;

	i = 0;
	do {
		to_send = min(size - i, MAX_CIPSEND_DATA);
		param.in.send_data.id = sock->conn_id;
		param.in.send_data.data.buff = ((uint8_t *)data) + i;
		param.in.send_data.data.len = to_send;
		ret = at_run_cmd(desc->at, AT_SEND, AT_SET_OP, &param);
		if (IS_ERR_VALUE(ret))
			return ret;

		i += to_send;
	} while (i < size);

	return (int32_t)size;
}

/** @brief See \ref network_interface.socket_recv */
static int32_t wifi_socket_recv(struct wifi_desc *desc, uint32_t sock_id,
				void *data, uint32_t size)
{
	struct socket_desc	*sock;
	uint32_t		available_size;
	int32_t			ret;

	if (!desc || sock_id >= MAX_CONNECTIONS || !size)
		return -EINVAL;

	/* TODO read data even if disconnected ? */
	sock = &desc->sockets[sock_id];
	if (sock->state != SOCKET_CONNECTED)
		return -ENOTCONN;

	cb_size(sock->cb, &available_size);
	if (available_size == 0)
		return -EAGAIN;

	size = min(available_size, size);
	ret = cb_read(sock->cb, data, size);
	if (IS_ERR_VALUE(ret))
		return ret;

	return size;
}

/** @brief See \ref network_interface.socket_sendto */
static int32_t wifi_socket_sendto(struct wifi_desc *desc, uint32_t sock_id,
				  const void *data, uint32_t size,
				  const struct socket_address to)
{
	UNUSED_PARAM(desc);
	UNUSED_PARAM(sock_id);
	UNUSED_PARAM(data);
	UNUSED_PARAM(size);
	UNUSED_PARAM(to);

	/* TODO: Implement for UDP */
	return FAILURE;
}

/** @brief See \ref network_interface.socket_recvfrom */
static int32_t wifi_socket_recvfrom(struct wifi_desc *desc, uint32_t sock_id,
				    void *data, uint32_t size,
				    struct socket_address *from)
{
	UNUSED_PARAM(desc);
	UNUSED_PARAM(sock_id);
	UNUSED_PARAM(data);
	UNUSED_PARAM(size);
	UNUSED_PARAM(from);

	/* TODO: Implement for UDP */
	return FAILURE;
}
