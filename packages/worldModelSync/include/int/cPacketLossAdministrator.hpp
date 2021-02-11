// Copyright 2015-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPacketLossAdministrator.hpp
 *
 *  Created on: Jul 16, 2015
 *      Author: Tim Kouters
 */

#ifndef CPACKETLOSSADMINISTRATOR_HPP_
#define CPACKETLOSSADMINISTRATOR_HPP_

#include <stdint.h>

class cPacketLossAdministrator
{
	public:
		cPacketLossAdministrator();
		~cPacketLossAdministrator();

		void notifyPacketNumber(uint8_t packetNumber, uint8_t robotID);
		float getPacketLoss(uint8_t robotID); // return as percentage

	private:
		static const uint8_t _PACKET_ARR_SIZE = 100;
		static const uint8_t _MAX_ROBOTS = 10;
		bool    _receivedPackages[_MAX_ROBOTS][_PACKET_ARR_SIZE];
		uint8_t _lastReceivedPacketNumber[_MAX_ROBOTS];
		bool    _initialized;

};


#endif /* CPACKETLOSSADMINISTRATOR_HPP_ */

