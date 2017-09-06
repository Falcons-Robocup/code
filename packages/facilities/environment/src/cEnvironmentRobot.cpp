 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cEnvironmentRobot.cpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Michel Koenen
 */

#include <ext/cEnvironmentCommon.hpp>
#include "FalconsCommon.h"

#include "ext/cEnvironmentRobot.hpp"
#include "json.h"
using namespace std;


cEnvironmentRobot::cEnvironmentRobot()
{
	getJSON();
}

cEnvironmentRobot::~cEnvironmentRobot()
{

}

float cEnvironmentRobot::getRadius()
{
	return _radius;
};

float cEnvironmentRobot::getRadiusMargin()
{
	return _radiusMargin;
};

/*! read the JSON file(s) as part of constructing phase and fill the private class members containing the values
 *
 */
void cEnvironmentRobot::getJSON()
{
	try
	{

		Json::Value root,robot;

		bool parsedSuccess = environmentCommon::readJSON( root);


		if(!parsedSuccess)
		{
			TRACE("Invalid JSON format read from cEnvironment.json file");
		    throw;
		}

		if( ! root.isMember("robot"))
		{
			TRACE("Cannot read robot information from cEnvironment.json file");
		    throw;
		}
		else
		{
			robot = root.get("robot", "ERROR");
		}

		if( robot.isMember("radius") )
		{
			// if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
			_radius = robot.get("radius", "ERROR").asFloat();
		}
		else
		{
			TRACE("JSON format issue with the JSON field 'robot.radius'");
			throw;
		}

		if( robot.isMember("radiusMargin") )
		{
			// if the supplied value does not match the expected format force ERROR by offering invalid type as default as well
			_radiusMargin = robot.get("radiusMargin", "ERROR").asFloat();
		}
		else
		{
			TRACE("JSON format issue with the JSON field 'robot.radiusMargin'");
			throw;
		}

	} catch (exception &e)
	{
    	printf("Invalid JSON format!");
    	TRACE("Invalid JSON format for reading environment field inputs");
		throw e;
	}
}
