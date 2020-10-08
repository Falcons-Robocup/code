 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
