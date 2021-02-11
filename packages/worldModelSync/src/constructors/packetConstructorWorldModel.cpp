// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * packetConstructorWorldModel.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/constructors/packetConstructorWorldModel.hpp"

#include <algorithm>

#include "int/configurators/configuratorWorldModelPacket.hpp"

#include "cDiagnostics.hpp"

packetConstructorWorldModel::packetConstructorWorldModel()
{
	_wmPacket.packetCounter = 0;
	_wmPacket.packetVersion = 2;
	_wmPacket.groupID = configuratorWorldModelPacket::getInstance().getGroupID();
	_wmPacket.robotID = getRobotNumber();

	for(size_t i = 0; i < NR_BALL_MEASUREMENTS; i++)
	{
		_wmPacket.balls[i].isValid = false;
	}

	for(size_t i = 0; i < NR_OBSTACLE_MEASUREMENTS; i++)
	{
		_wmPacket.obstacles[i].isValid = false;
	}
}

packetConstructorWorldModel::~packetConstructorWorldModel()
{

}

void packetConstructorWorldModel::setPacketCounter(const uint8_t value)
{
	_wmPacket.packetCounter = value;
}

void packetConstructorWorldModel::setPacketVersion(const uint8_t value)
{
	_wmPacket.packetVersion = value;
}

void packetConstructorWorldModel::setGroupID(const uint8_t value)
{
	_wmPacket.groupID = value;
}

void packetConstructorWorldModel::setRobotID(const uint8_t value)
{
	_wmPacket.robotID = value;
}

void packetConstructorWorldModel::setBallMeasurements(std::vector<ballMeasurement> measurements)
{
	/*
	 * Ensure newest measurements are added first
	 *
	 * TODO: ticket #612: this is going to be a problem for multiCam
	 * at any time instant, vision will produce say 10 candidates, most of which are false positives (lights, T-shirts, etc)
	 * with a fixed buffer size of 2, odds are large that the actual ball is not synced to other team members
	 * NOTE: selection & sorting in worldModel: see worldModel/src/administrators/ballAdministrator.cpp::getLocalBallMeasurements
	 */
	std::sort(measurements.begin(), measurements.end(), sortBallMeasurementsOnCameraTypeAndTimeStamp);


	size_t i;
	for(i = 0; ((i < measurements.size()) && (i < NR_BALL_MEASUREMENTS)); i++)
	{
		_wmPacket.balls[i] = measurements.at(i);
		_wmPacket.balls[i].isValid = true;
	}
	for(; i < NR_BALL_MEASUREMENTS; i++)
	{
		_wmPacket.balls[i].isValid = false;
	}
}

void packetConstructorWorldModel::setBallPossession(const bool hasBallPossession)
{
	_wmPacket.ballPossession.hasBallPossession = hasBallPossession;
}

void packetConstructorWorldModel::setObstacleMeasurements(std::vector<obstacleMeasurement> measurements)
{
	/*
	 * Ensure newest measurements are added first
	 */
	std::sort(measurements.begin(), measurements.end(), sortObstacleMeasurementsOnTimeStamp);

	size_t i;
	for(i = 0; ((i < measurements.size()) && (i < NR_OBSTACLE_MEASUREMENTS)); i++)
	{
		_wmPacket.obstacles[i] = measurements.at(i);
		_wmPacket.obstacles[i].isValid = true;
	}
	for(; i < NR_OBSTACLE_MEASUREMENTS; i++)
	{
		_wmPacket.obstacles[i].isValid = false;
	}
}

void packetConstructorWorldModel::setRobotLocation(const robotLocationStructure location)
{
	_wmPacket.robotPosition = location;
}

void packetConstructorWorldModel::setByteArray(Facilities::cByteArray byteArray)
{
	std::vector<uint8_t> array;
	byteArray.getData(array);
	packetStructureWorldModel *pd = reinterpret_cast<packetStructureWorldModel*>(array.data());
	_wmPacket = (*pd);
}

Facilities::cByteArray packetConstructorWorldModel::getByteArray()
{
	std::vector<uint8_t> array;
	Facilities::cByteArray retArray;

	uint8_t *pd = reinterpret_cast<uint8_t *>(&_wmPacket);
    array.insert(array.end(), pd, pd + sizeof(_wmPacket));
    retArray.setData(array);
    return retArray;
}

packetStructureWorldModel packetConstructorWorldModel::getWorldModelStructure() const
{
	return _wmPacket;
}
