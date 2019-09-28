// Copyright 2017 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef TCP_SOCKET_HPP
#define TCP_SOCKET_HPP

#include <cstdint>
#include <string>
#include <sys/socket.h>

#include "zynqGrabConfig.hpp"

// TODO: align with MTU size
// Likely the OS / driver does the magic with the Ethernet buffer
// But it might make sense to do some timing tests
#define MAX_RECEIVE_SIZE 1500
#define MAX_TRANSMIT_SIZE 1500

union tcp_payload_t {
	// payload size maximal 16MiB
	uint8_t u8[1 << 24]; // unsigned payload
	uint16_t u16[1 << 23];
	uint32_t u32[1 << 22];
	uint64_t u64[1 << 21];
	int8_t s8[1 << 24]; // signed payload
	int16_t s16[1 << 23];
	int32_t s32[1 << 22];
	int64_t s64[1 << 21];
	float f32[1 << 22];
	double d64[1 << 21];
};

typedef struct {
	uint8_t id;
	uint8_t cnt;
	uint16_t placeholder0;
	uint32_t size; // packet size in bytes, including the size of the header
	tcp_payload_t pl;
}__attribute__((packed)) tcp_packet_t;

class tcp_socket {
private:
	// ## conf ##
	int camIndex;
	zynqGrabConfig *config;
	bool update;

	// ## setup ##
	struct sockaddr_in server_addr;
	int listen_sock_fd, sock_fd;

	// ## receive ##
	tcp_packet_t rx_packet;
	char rx_buffer[1024]; // TODO: increase because MTU size is likely around 1400 bytes
	size_t rx_buffer_offset;
	ssize_t rx_buffer_available;
	ssize_t rx_buffer_size;
	uint8_t rx_packet_cnt_expected;

	bool rx_buffer_refill();

	// ## send ##
	tcp_packet_t tx_packet;

	bool tx_packet_send();


public:
	// ## setup ##
	tcp_socket(int camIndex, zynqGrabConfig *config); // constructor

	void bind(uint16_t port); // used by server
	void wait_for_client(); // used by server
	void connect(std::string ip, uint16_t port); // used by client
	void disconnect();

	// ## packet information ##
	size_t get_header_size() {
		return (sizeof(tcp_packet_t) - sizeof(tcp_payload_t));
	}

	size_t get_max_packet_size() {
		return (sizeof(tcp_packet_t));
	}

	// ## receive ##
	bool receive();
	bool get_rx_packet_cnt_valid();

	uint8_t get_rx_packet_id() {
		return rx_packet.id;
	}

	uint32_t get_rx_packet_size() {
		return rx_packet.size;
	}

	ssize_t get_rx_pl_size() {
		return (rx_packet.size - get_header_size());
	}

	uint32_t get_rx_pl_u32(size_t index) {
		return rx_packet.pl.u32[index];
	}

	uint64_t get_rx_pl_u64(size_t index) {
		return rx_packet.pl.u64[index];
	}

	bool get_rx_pl(uint8_t* buffer, size_t buffer_size, uint32_t &rx_pl_size);

	// ## send ##
	bool send(uint8_t id);
	bool send(uint8_t id, uint32_t pl);
	bool send(uint8_t id, uint64_t pl);
	bool send(uint8_t id, uint8_t* buffer, uint32_t buffer_size);

	void stop() {
		update = false;
	}

	void serverUpdate();

};

#endif
