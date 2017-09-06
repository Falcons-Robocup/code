 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 *  ballPossessionType.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#include "int/types/ball/ballPossessionType.hpp"

#include <stdexcept>
#include <iostream>

#include "cEnvironmentField.hpp"
#include "cDiagnosticsEvents.hpp"
#include "timeConvert.hpp"
#include "tracer.hpp"
#include "FalconsCommon.h"

#define BALLCAMERATIMEOUT (0.5)

ballPossessionClass_t::ballPossessionClass_t()
{
	_robotID = 0;
	_ballIsClaimedByBallHandlers = false;
	_ballIsSeenByCameraTimeStamp = 0.0;
	_lastResult = false;
	_claimedPositionX = 0.0;
	_claimedPositionY = 0.0;
	_ballCloseToObstacle = false;
}

ballPossessionClass_t::ballPossessionClass_t(const uint8_t robotID)
{
	_robotID = robotID;
	_ballIsClaimedByBallHandlers = false;
	_ballIsSeenByCameraTimeStamp = 0.0;
	_lastResult = false;
	_claimedPositionX = 0.0;
	_claimedPositionY = 0.0;
	_ballCloseToObstacle = false;
}

ballPossessionClass_t::~ballPossessionClass_t()
/*
 * Death once had a near-Chuck-Norris experience
 */
{

}

bool ballPossessionClass_t::getBallCloseToObstacle() const
{
    return _ballCloseToObstacle;
}

void ballPossessionClass_t::setBallCloseToObstacle(bool b)
{
    _ballCloseToObstacle = b;
}

void ballPossessionClass_t::claimBallByBallHandlers(const float x, const float y)
{
    //TRACE("> x=%6.2f y=%6.2f robot=%d", x, y, (int)_robotID);
	try
	{
		if(cEnvironmentField::getInstance().isPositionInArea(x,y, areaName::A_FIELD_SAFETY_BOUNDARIES))
		{
	        // trace only state changes
	        if (!_ballIsClaimedByBallHandlers)
	        {
	            TRACE("entering ballHandler ball possession for robot %d", (int)_robotID);
	        }
			_ballIsClaimedByBallHandlers = true;
			_claimedPositionX = x;
			_claimedPositionY = y;
		}
		else
		{
			TRACE("Ignored ball claimed at %6.2f, %6.2f", x, y);
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballPossessionClass_t::claimBallByCamera()
{
    // this function is called actively when vision sees the ball
    // if vision does not see the ball, then releaseBallByCamera is NOT called, 
    // we need to be robust for vision glitches, so we use an internal timer
    TRACE(">");
	try
	{
		_ballIsSeenByCameraTimeStamp = getTimeNow();
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}


}

void ballPossessionClass_t::releaseBallByCamera() // never called!!
{
    TRACE(">");
	try
	{
		_ballIsSeenByCameraTimeStamp = 0.0;
	    // trace only state changes
//	    if (_ballIsSeenByCamera)
//	    {
//	        TRACE("releasing camera ball possession for robot %d", (int)_robotID);
//	    }
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballPossessionClass_t::releaseBallByBallHandlers()
{
    TRACE(">");
	try
	{
	    // trace only state changes
	    if (_ballIsClaimedByBallHandlers)
	    {
	        TRACE("releasing ballHandler ball possession for robot %d", (int)_robotID);
	    }
		_ballIsClaimedByBallHandlers = false;
		_ballIsSeenByCameraTimeStamp = 0.0;
		_claimedPositionX = 0.0;
		_claimedPositionY = 0.0;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool ballPossessionClass_t::hasBallPossession()
/*!
 * \brief Returns true if ball is claimed
 * Both the camera and the handlers must see the ball
 */
{
    TRACE("> _ballIsClaimedByBallHandlers=%d _ballIsSeenByCameraTimeStamp=%f", _ballIsClaimedByBallHandlers, _ballIsSeenByCameraTimeStamp);
	try
	{
	    bool currentResult = ballHandlersClaimedPossession() && visionClaimedPossession();
	    _lastResult = currentResult;
		return currentResult;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballPossessionClass_t::getClaimedPosition(float &x, float &y)
{
	try
	{
		x = _claimedPositionX;
		y = _claimedPositionY;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}


}

uint8_t ballPossessionClass_t::getRobotID() const
{
	try
	{
		return _robotID;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool ballPossessionClass_t::ballHandlersClaimedPossession() const
{
	try
	{
		return _ballIsClaimedByBallHandlers;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
bool ballPossessionClass_t::visionClaimedPossession() const
{
	try
	{
		return getTimeNow() - _ballIsSeenByCameraTimeStamp < BALLCAMERATIMEOUT;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
