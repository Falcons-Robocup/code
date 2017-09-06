/*
 * Package.c
 *
 *  Created on: Jul 9, 2017
 *      Author: Edwin Schreuder
 */

#include <stdbool.h>
#include <string.h>

#include "Package.h"
#include "UART.h"

#define SOF (0x5a)

static uint8_t package_calculate_checksum(Package * package);
static bool package_find_from_data(uint8_t data, Package * package);

//! Transmits a package.
//! \details	Wraps the payload in a package formats and sends it over uart.
//! \author		Edwin Schreuder
void Package_send(Package * package)
{
	size_t index;

	// Send the SOF.
	UART_putchar(SOF);

	// Send the package.
	for (index = 0; index < sizeof(Package); index++) {
		UART_putchar(((uint8_t *) package)[index]);
	}

	// Send the checksum.
	UART_putchar(package_calculate_checksum(package));
}

//! Receives a package.
//! \author		Edwin Schreuder
bool Package_receive(Package * package)
{
	bool package_received = false;

	while (UART_has_data() && !package_received) {
		uint8_t data = UART_getchar();

		package_received = package_find_from_data(data, package);
	}

	return package_received;
}

//! Adds the supplied data to the data and returns the package when completed.
//! \returns	A reference to the package is returned if the package is available.
//!				The reference is valid until this function is called again.
//!				If no package is available, NULL is returned.
//! \author		Edwin Schreuder
bool package_find_from_data(uint8_t data, Package * package)
{
	bool package_received = false;

	static uint8_t buffer[sizeof(Package) + 1] = {0};
	static uint8_t buffer_length = 0;
	static bool received_sof = false;

	if (received_sof) {
		buffer[buffer_length] = data;
		buffer_length++;

		if (buffer_length > sizeof(Package)) {
			// We have reached the checksum field. Set the package state to RECEIVING_SOF.
			received_sof = false;

			// Verify that the checksum matches.
			if (package_calculate_checksum((Package *) buffer) == data) {
				// The checksum matches, so return the package.
				package_received = true;
				buffer_length = 0;
			}
			else {
				// Something is wrong, because the checksum does not match.
				// To recover, we can look at the current buffer and see if there is another SOF.
				while ((buffer_length > 0) && (!received_sof)) {
					buffer_length--;
					if (buffer[sizeof(buffer) - buffer_length - 1] == SOF) {
						memcpy(buffer, &(buffer[sizeof(buffer) - buffer_length]), buffer_length);
						received_sof = true;
					}
				}
			}
		}
	}
	else {
		received_sof = (data == SOF);
	}

	if (package_received) {
		memcpy(package, buffer, sizeof(Package));
	}

	return package_received;
}

//! Calculates the checksum of a package.
//! \details	The checksum is calculated by adding all values and taking the inverse.
//! \author		Edwin Schreuder
uint8_t package_calculate_checksum(Package * package)
{
	uint8_t checksum = 0x00;
	size_t index;
	for (index = 0; index < sizeof(Package); index++) {
		checksum += ((uint8_t *) package)[index];
	}

	return ~checksum;
}

