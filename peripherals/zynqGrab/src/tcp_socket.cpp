// Copyright 2017 Andre Pool
// SPDX-License-Identifier: Apache-2.0
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include "tcp_socket.hpp"

using namespace std;

tcp_socket::tcp_socket(int camIndex, zynqGrabConfig *config = NULL) {
	this->camIndex = camIndex;
	this->config = config;

	update = true;

	memset((char *) &server_addr, 0, sizeof(server_addr));
	listen_sock_fd = 0;
	sock_fd = 0;

	rx_packet.id = 0;
	rx_packet.cnt = 0;
	rx_packet.size = 0;
	rx_packet.placeholder0 = 0;

	rx_packet_cnt_expected = 0;
	rx_buffer_offset = 0;
	rx_buffer_available = 0;
	rx_buffer_size = 0;

	tx_packet.id = 0;
	tx_packet.cnt = 0;
	tx_packet.size = 0;
	tx_packet.placeholder0 = 0;
}

// used by server
void tcp_socket::bind(uint16_t port) {
	printf("INFO    : cam %d TCP bind socket to server\n", camIndex);
	// create a TCP socket
	listen_sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (listen_sock_fd < 0) {
		printf("ERROR   : cam %d cannot create TCP socket, message: %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}

	// setup the locale socket
	server_addr.sin_family = AF_INET; // IP protocol family

#if defined(__arm__)
	// the zynq boards can use their Ethernet address
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // accept any incoming messages
#else
	// for x86 testing use additional created Ethernet addresses on the lo interface
	if( camIndex == 0 ) {
		server_addr.sin_addr.s_addr = inet_addr("127.0.0.70");
	} else if( camIndex == 1 ) {
		server_addr.sin_addr.s_addr = inet_addr("127.0.0.71");
	} else if( camIndex == 2 ) {
		server_addr.sin_addr.s_addr = inet_addr("127.0.0.72");
	} else if( camIndex == 3 ) {
		server_addr.sin_addr.s_addr = inet_addr("127.0.0.73");
	} else {
		printf("ERROR  : cam %d invalid camera index %u\n", camIndex, camIndex );
		exit(EXIT_FAILURE);
	}
#endif

	server_addr.sin_port = htons(port); // port

	// bind the socket to the local server address and port
	if (::bind(listen_sock_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
		printf("ERROR   : cam %d cannot bind to IP %s port %u, message: %s\n", camIndex, inet_ntoa(server_addr.sin_addr),
				ntohs(server_addr.sin_port), strerror(errno));
		exit(EXIT_FAILURE);
	}
	printf("INFO    : cam %d TCP connection local IP %s port %u\n", camIndex, inet_ntoa(server_addr.sin_addr),
			ntohs(server_addr.sin_port));

}

// used by server
void tcp_socket::wait_for_client() {
	if (listen(listen_sock_fd, 5)) {
		printf("ERROR   : cam %d cannot listen on IP %s port %u, message: %s\n", camIndex, inet_ntoa(server_addr.sin_addr),
				ntohs(server_addr.sin_port), strerror(errno));
		exit(EXIT_FAILURE);
	}

	// accept the connection to the socket
	struct sockaddr_in client_addr;
	socklen_t client_length = sizeof(client_addr);
	sock_fd = accept(listen_sock_fd, (struct sockaddr *) &client_addr, &client_length);
	if (sock_fd < 0) {
		printf("ERROR   : cam %d cannot accept connection from remote IP %s port %u, message: %s\n",
				camIndex, inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), strerror(errno));
		exit(EXIT_FAILURE);
	}
	printf("INFO    : cam %d TCP connection remote IP %s port %u\n", camIndex, inet_ntoa(client_addr.sin_addr),
			ntohs(client_addr.sin_port));

	rx_packet_cnt_expected = 0; // synchronize packet counter from client to server
	tx_packet.cnt = 0; // synchronize packet counter from server to client
}

// used by client
void tcp_socket::connect(string ip, uint16_t port) {
	printf("INFO    : cam %d setup connection to server\n", camIndex);
	// create a TCP socket
	sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock_fd < 0) {
		printf("ERROR   : cam %d cannot create TCP socket, message: %s\n", camIndex, strerror(errno));
		exit(EXIT_FAILURE);
	}

	// determine if server IP is valid
	struct hostent *server;
	server = gethostbyname(ip.c_str());
	if (server == NULL) {
		printf("ERROR   : cam %d server IP %s does not exist, message: %s\n", camIndex, ip.c_str(), strerror(errno));
		exit(EXIT_FAILURE);
	}

	// fill in server_addr struct
	server_addr.sin_family = AF_INET; // IP protocol family
	memcpy((char *) &server_addr.sin_addr.s_addr, (char *) server->h_addr,
	server->h_length); // set server address : server_addr.sin_addr.s_addr = server->h_addr;
	server_addr.sin_port = htons(port); // set server port

	// connect to remote server
	if (::connect(sock_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
		printf("ERROR   : cam %d cannot connect to server IP %s port %u, message: %s\n", camIndex, inet_ntoa(server_addr.sin_addr),
				ntohs(server_addr.sin_port), strerror(errno));
		exit(EXIT_FAILURE);
	}
	printf("INFO    : cam %d TCP connection to IP %s port %u\n", camIndex, inet_ntoa(server_addr.sin_addr), ntohs(server_addr.sin_port));

	rx_packet_cnt_expected = 0; // synchronize packet counter from server to client
	tx_packet.cnt = 0; // synchronize packet counter from client to server
}

void tcp_socket::disconnect() {
	printf("INFO    : cam %d close socket\n", camIndex);
	close(sock_fd);
}

// ## receive ##

bool tcp_socket::rx_buffer_refill() {
	if (rx_buffer_available > 0) {
		printf("WARNING : cam %d remove trailing %zd bytes from receive buffer\n", camIndex, rx_buffer_available);
	}

	rx_buffer_size = read(sock_fd, rx_buffer, sizeof(rx_buffer));
	// set the buffer pointer and size of the current amount of data available in the buffer
	rx_buffer_offset = 0;
	rx_buffer_available = rx_buffer_size;
	if (rx_buffer_size < 0) {
		printf("ERROR   : cam %d cannot read data from socket, message: %s\n", camIndex, strerror(errno));
		close(sock_fd);
		exit(EXIT_FAILURE);
	} else if (rx_buffer_size == 0) {
		printf("WARNING : cam %d empty packet received\n", camIndex);
		return false;
	}
	return true;
}

bool tcp_socket::receive() {
	bool keep_searching = true;
	bool header_found = false;
	bool connected = true;
	size_t received_size = 0;

	// clear the header of the receive packet struct, so the rx_packet.size can be used on a higher level to indicate an empty packet was received
	memset((char *) &rx_packet, 0, get_header_size());

	while (keep_searching && connected) {
		// is there still data in the buffer of the previous read?
		while ((rx_buffer_available <= 0) && keep_searching && connected) {
			connected = rx_buffer_refill(); // this will block (read) until data is available
		}

		if (rx_buffer_available > 0) {
			if (!header_found) {
				// get packet header
				if (rx_buffer_available >= (ssize_t) get_header_size()) {
					// copy the header from buffer to the receive packet struct
					memcpy(&rx_packet, rx_buffer + rx_buffer_offset, get_header_size());

					// update the buffer pointer and the size of the remaining data left in the buffer
					rx_buffer_offset += get_header_size();
					rx_buffer_available -= get_header_size();
					received_size = get_header_size();
					header_found = true;
				} else {
					printf("WARNING : cam %d wait for packet header, received only %zd bytes while need a least %zu bytes\n",
							camIndex, rx_buffer_available, get_header_size());
				}
			}
			if (header_found) {
				// complete the payload
				if (rx_packet.size > sizeof(rx_packet)) {
					printf(
							"ERROR   : cam %d expected received packet size is %u bytes, while maximal packet size is %zu bytes\n",
							camIndex, rx_packet.size, sizeof(rx_packet));
					keep_searching = false;
				} else if (received_size < rx_packet.size) { // received < expected
					// the packet is not yet complete
					// printf("INFO    : cam %d payload, received size %3zd, packet size %3u, available %3zd\n", camIndex, received_size, rx_packet.size,
					//		rx_buffer_available);

					// determine how many bytes can be copied
					ssize_t copy_size = rx_packet.size - received_size; // size = expected - "already received"
					if (copy_size > rx_buffer_available) {
						copy_size = rx_buffer_available; // we might need more data then available in the buffer
					}

					// copy a "section" of the payload from the buffer to the packet struct
					memcpy(&rx_packet.pl.u8[received_size - get_header_size()], rx_buffer + rx_buffer_offset,
							copy_size);

					// update the amount of bytes we have received
					received_size += copy_size;

					// update the buffer pointer and the size of the remaining data left in the buffer
					rx_buffer_offset += copy_size;
					rx_buffer_available -= copy_size;
				}

				// check if packet complete
				if (received_size == rx_packet.size) {
					// all bytes are now available in the packet
					keep_searching = false;
				}
			}
		}
	}
	return connected;
}

bool tcp_socket::get_rx_packet_cnt_valid() {
	if (rx_packet.cnt != rx_packet_cnt_expected) {
		printf("ERROR   : cam %d expected receive packet counter %u, but got %u\n", camIndex, rx_packet_cnt_expected, rx_packet.cnt);
		rx_packet_cnt_expected = rx_packet.cnt + 1;
		return false;
	} else {
		rx_packet_cnt_expected++;
	}
	return true;
}

bool tcp_socket::get_rx_pl(uint8_t* buffer, size_t buffer_size, uint32_t &rx_pl_size) {
	if ((ssize_t) buffer_size < get_rx_pl_size()) {
		printf("ERROR   : cam %d buffer of %zu bytes cannot hold payload of %zd bytes\n", camIndex, buffer_size, get_rx_pl_size());
		// prevent issues with availability of previous data
		memset(buffer, 0, buffer_size);
		rx_pl_size = 0;
		return false;
	} else {
		memcpy(buffer, &rx_packet.pl, get_rx_pl_size());
		rx_pl_size = get_rx_pl_size();
	}
	return true;
}

// ## send ##
bool tcp_socket::tx_packet_send() {
	if (tx_packet.size > sizeof(tx_packet)) {
		printf("ERROR   : cam %d send id %u requested to send %u bytes, while the maximal packet size is only %zu bytes\n",
				camIndex, tx_packet.id, tx_packet.size, sizeof(tx_packet));
		return false;
	}

	size_t written_size = 0;
	while (written_size < tx_packet.size) {
		size_t proposed_size = tx_packet.size - written_size;
		if (proposed_size > MAX_TRANSMIT_SIZE) {
			proposed_size = MAX_TRANSMIT_SIZE;
		}

		// update the pointer in the tx_packet used by the write socket
		char *tx_packet_ptr = written_size + (char *) &tx_packet;
		ssize_t actual_size = write(sock_fd, tx_packet_ptr, proposed_size);
		if (actual_size < 0) {
			printf("ERROR   : cam %d cannot write to socket, message: %s\n", camIndex, strerror(errno));
			exit(EXIT_FAILURE);
		}

		// printf(" written %10zu proposed %6zu actual %6zd packet %10u\n", written_size, proposed_size, actual_size,
		//		tx_packet.size);
		written_size += actual_size;
	}

	if (tx_packet.size != written_size) {
		printf("ERROR   : cam %d send id %2d only %zu bytes of %u bytes have been send", camIndex, tx_packet.id, written_size,
				tx_packet.size);
		exit(EXIT_FAILURE);
	}
	return true;
}

bool tcp_socket::send(uint8_t id, uint32_t pl) {
	tx_packet.id = id;
	tx_packet.size = get_header_size() + sizeof(pl);
	tx_packet.pl.u32[0] = pl;
	bool status = tx_packet_send();
	tx_packet.cnt++; // wrap around at 255
	return status;
}

bool tcp_socket::send(uint8_t id, uint64_t pl) {
	tx_packet.id = id;
	tx_packet.size = get_header_size() + sizeof(pl);
	tx_packet.pl.u64[0] = pl;
	bool status = tx_packet_send();
	tx_packet.cnt++; // wrap around at 255
	return status;
}

bool tcp_socket::send(uint8_t id) {
	tx_packet.id = id;
	tx_packet.size = get_header_size();
	bool status = tx_packet_send();
	tx_packet.cnt++; // wrap around at 255
	return status;
}

bool tcp_socket::send(uint8_t id, uint8_t* buffer, uint32_t buffer_size) {
	uint32_t tx_packet_size = get_header_size() + buffer_size;
	if ((size_t) tx_packet_size > get_max_packet_size()) {
		printf("WARNING: cam %d request to sent %u bytes, but truncated to maximal packet length of %zu bytes\n",
				camIndex, tx_packet_size, get_max_packet_size());
		tx_packet_size = get_max_packet_size();
	}

	tx_packet.id = id;
	tx_packet.size = get_header_size() + buffer_size;
	memcpy(&tx_packet.pl.u8[0], buffer, buffer_size);
	bool status = tx_packet_send();
	tx_packet.cnt++; // wrap around at 255
	return status;
}

void tcp_socket::serverUpdate() {
	while (update) {
		wait_for_client();

		bool connected = true;
		while (connected) {
			// get_packet();
			connected = receive();
			uint32_t rx_packet_id = get_rx_packet_id();
			// printf("received packet id %u, %d\n", rx_packet_id, get_rx_pl_u32(0));
			uint32_t value = get_rx_pl_u32(0);
			if (rx_packet_id == 10) {
				config->setLineValMin(value);
			} else if (rx_packet_id == 11) {
				config->setLineSatMax(value);
			} else if (rx_packet_id == 12) {
				config->setBallValMin(value);
			} else if (rx_packet_id == 13) {
				config->setBallSatMin(value);
			} else if (rx_packet_id == 14) {
				config->setBallHueMin(value);
			} else if (rx_packet_id == 15) {
				config->setBallHueMax(value);
			} else if (rx_packet_id == 16) {
				config->setObstacleValMax(value);
			} else if (rx_packet_id == 17) {
				config->setObstacleSatMax(value);
			} else if (rx_packet_id == 18) {
				config->setRed(value);
			} else if (rx_packet_id == 19) {
				config->setGreen(value);
			} else if (rx_packet_id == 20) {
				config->setBlue(value);
			} else if (rx_packet_id == 21) {
				config->setTestPattern((uint8_t) value);
			} else if (rx_packet_id == 22) {
				config->setAnalogGain((uint8_t) value);
			} else if (rx_packet_id == 23) {
				config->setLines((uint16_t) value);
			} else if (rx_packet_id == 24) {
				config->setPixels((uint16_t) value);
			} else if (rx_packet_id == 25) {
				config->setXStart((uint16_t) value);
			} else if (rx_packet_id == 26) {
				config->setXEnd((uint16_t) value);
			} else if (rx_packet_id == 27) {
				config->setXSize((uint16_t) value);
			} else if (rx_packet_id == 28) {
				config->setYStart((uint16_t) value);
			} else if (rx_packet_id == 29) {
				config->setYEnd((uint16_t) value);
			} else if (rx_packet_id == 30) {
				config->setYSize((uint16_t) value);
			} else if (rx_packet_id == 31) {
				config->setEXCK_FREQ(value);
			} else if (rx_packet_id == 32) {
				config->setVTPXCK_DIV(value);
			} else if (rx_packet_id == 33) {
				config->setVTSYCK_DIV(value);
			} else if (rx_packet_id == 34) {
				config->setPREPLLCK_VT_DIV(value);
			} else if (rx_packet_id == 35) {
				config->setPREPLLCK_OP_DIV(value);
			} else if (rx_packet_id == 36) {
				config->setPLL_VT_MPY(value);
			} else if (rx_packet_id == 37) {
				config->setOPPXCK_DIV(value);
			} else if (rx_packet_id == 38) {
				config->setOPSYCK_DIV(value);
			} else if (rx_packet_id == 39) {
				config->setPLL_OP_MPY(value);
			} else if (rx_packet_id == 43) {
				config->setCameraReset(); // the camera reset is auto clear
			} else if (rx_packet_id == 44) {
				config->setFloorValMin(value);
			} else if (rx_packet_id == 45) {
				config->setFloorSatMin(value);
			} else if (rx_packet_id == 46) {
				config->setFloorHueMin(value);
			} else if (rx_packet_id == 47) {
				config->setFloorHueMax(value);
			} else if (rx_packet_id == 48) {
				config->setLineTransferPixelsMax(value);
			} else if (rx_packet_id == 49) {
				config->setLineFloorWindowSize(value);
			} else if (rx_packet_id == 50) {
				config->setLineFloorPixelsMin(value);
			} else if (rx_packet_id == 51) {
				config->setLineWindowSize(value);
			} else if (rx_packet_id == 52) {
				config->setLinePixelsMin(value);
			} else if (rx_packet_id == 53) {
				config->setBallWindowSize(value);
			} else if (rx_packet_id == 54) {
				config->setBallPixelsMin(value);
			} else if (rx_packet_id == 55) {
				config->setBallFalsePixelsMax(value);
			} else if (rx_packet_id == 56) {
				config->setObstacleFloorWindowSize(value);
			} else if (rx_packet_id == 57) {
				config->setObstacleLineWindowSize(value);
			} else if (rx_packet_id == 58) {
				config->setObstacleBallWindowSize(value);
			} else if (rx_packet_id == 59) {
				config->setObstacleWindowSize(value);
			} else if (rx_packet_id == 60) {
				config->setObstacleTransferPixelsMax(value);
			} else if (rx_packet_id == 61) {
				config->setObstacleFloorPixelsMin(value);
			} else if (rx_packet_id == 62) {
				config->setObstacleLinePixelsMin(value);
			} else if (rx_packet_id == 63) {
				config->setObstacleBallPixelsMin(value);
			} else if (rx_packet_id == 64) {
				config->setObstaclePixelsMin(value);
			} else if (rx_packet_id == 65) {
				config->setShutter(value);
			} else {
				printf("ERROR   : cam %d unknown packet ID %d\n", camIndex, rx_packet_id);
			}
		}
		printf("INFO    : cam %d wait before disconnect\n", camIndex);
		usleep(500000);
		disconnect();
	}
}
