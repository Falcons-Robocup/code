// Copyright 2016-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * packetConstructorWorldModel.hpp
 *
 *  Created on: Oct 9, 2016
 *      Author: Tim Kouters
 */

#ifndef PACKETCONSTRUCTORWORLDMODEL_HPP_
#define PACKETCONSTRUCTORWORLDMODEL_HPP_

#include <vector>

#include "int/types/packetStructureWorldModel.hpp"

#include "cByteArray.hpp"

class packetConstructorWorldModel
{
	public:
		packetConstructorWorldModel();
		~packetConstructorWorldModel();

		void setPacketCounter(const uint8_t value);
		void setPacketVersion(const uint8_t value);
		void setGroupID(const uint8_t value);
		void setRobotID(const uint8_t value);
		void setBallMeasurements(std::vector<ballMeasurement> measurements);
		void setBallPossession(const bool hasBallPossession);
		void setObstacleMeasurements(std::vector<obstacleMeasurement> measurements);
		void setRobotLocation(const robotLocationStructure location);
		void setByteArray(Facilities::cByteArray byteArray);

		Facilities::cByteArray getByteArray();
		packetStructureWorldModel getWorldModelStructure() const;

	private:
		packetStructureWorldModel _wmPacket;
};

#endif /* PACKETCONSTRUCTORWORLDMODEL_HPP_ */
