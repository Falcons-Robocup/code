 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * objectMeasurementType.cpp
 *
 *  Created on: Jan 10, 2017
 *      Author: Jan Feitsma
 */

#include "int/types/object/objectMeasurementType.hpp"

objectMeasurementType::objectMeasurementType()
{
	_timestamp = 0.0;
	_confidence = 0.0;
	_cameraType = cameraType::INVALID;
	_azimuth = 0.0;
	_elevation = 0.0;
	_radius = 0.0;
	_cameraX = 0.0;
	_cameraY = 0.0;
	_cameraZ = 0.0;
	_cameraPhi = 0.0;
}

objectMeasurementType::~objectMeasurementType()
/*
 * Chuck Norris doesn't drive cars, he flies trains
 */
{

}

void objectMeasurementType::setID(const uniqueWorldModelID id)
{
	_identifier = id;
}

void objectMeasurementType::setTimestamp(const double stamp)
{
	_timestamp = stamp;
}

void objectMeasurementType::setCameraType(const cameraType type)
{
	_cameraType = type;
}

void objectMeasurementType::setConfidence(const float confidence)
{
	_confidence = confidence;
}

void objectMeasurementType::setSphericalCoords(const float azimuth, const float elevation, const float radius)
{
	_azimuth = azimuth;
	_elevation = elevation;
	_radius = radius;
}

void objectMeasurementType::setCameraOffset(const float camX, const float camY, const float camZ, const float camPhi)
{
	_cameraX = camX;
	_cameraY = camY;
	_cameraZ = camZ;
	_cameraPhi = camPhi;
}

uniqueWorldModelID objectMeasurementType::getID() const
{
	return _identifier;
}

double objectMeasurementType::getTimestamp() const
{
	return _timestamp;
}

cameraType objectMeasurementType::getCameraType() const
{
	return _cameraType;
}

float objectMeasurementType::getConfidence() const
{
	return _confidence;
}

float objectMeasurementType::getAzimuth() const
{
	return _azimuth;
}

float objectMeasurementType::getElevation() const
{
	return _elevation;
}

float objectMeasurementType::getRadius() const
{
	return _radius;
}

float objectMeasurementType::getCameraX() const
{
	return _cameraX;
}

float objectMeasurementType::getCameraY() const
{
	return _cameraY;
}

float objectMeasurementType::getCameraZ() const
{
	return _cameraZ;
}

float objectMeasurementType::getCameraPhi() const
{
	return _cameraPhi;
}

