/*
 * Package.h
 *
 *  Created on: Jul 9, 2017
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_PACKAGE_H_
#define INCLUDE_PACKAGE_H_

#include "config.h"

typedef struct {
	uint8_t command;
	uint8_t payload[PACKAGE_PAYLOAD_LENGTH];
} Package;

void Package_send(Package * package);
bool Package_receive(Package * package);

#endif /* INCLUDE_PACKAGE_H_ */
