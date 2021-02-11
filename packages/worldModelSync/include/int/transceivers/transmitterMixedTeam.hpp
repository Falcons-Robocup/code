// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * transmitterMixedTeam.hpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#ifndef TRANSMITTERMIXEDTEAM_HPP_
#define TRANSMITTERMIXEDTEAM_HPP_

#include "cTransmitterUDP.hpp"
#include "cByteArray.hpp"

class transmitterMixedTeam
{
	public:
		transmitterMixedTeam();
		~transmitterMixedTeam();

		void reconnect();
		void sendPacket(Facilities::cByteArray wmPacket);
	private:
		Facilities::Network::cTransmitterUDP *_udpTransmitter;

		void cleanUpTransmitter();
};

#endif /* TRANSMITTERMIXEDTEAM_HPP_ */
