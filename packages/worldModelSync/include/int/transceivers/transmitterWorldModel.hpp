// Copyright 2016-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * transmitterWorldModel.hpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#ifndef TRANSMITTERWORLDMODEL_HPP_
#define TRANSMITTERWORLDMODEL_HPP_

#include "cTransmitterUDP.hpp"
#include "cByteArray.hpp"

class transmitterWorldModel
{
	public:
		transmitterWorldModel();
		~transmitterWorldModel();

		void reconnect();
		void sendPacket(Facilities::cByteArray wmPacket);
	private:
		Facilities::Network::cTransmitterUDP *_udpTransmitter;

		void cleanUpTransmitter();
};

#endif /* TRANSMITTERWORLDMODEL_HPP_ */
