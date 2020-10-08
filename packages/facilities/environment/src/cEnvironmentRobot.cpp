 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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
#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "ext/cEnvironmentRobot.hpp"
using namespace std;


cEnvironmentRobot::cEnvironmentRobot()
{
	getConfig();
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

/*! read the YAML file(s) as part of constructing phase and fill the private class members containing the values
 *
 */
void cEnvironmentRobot::getConfig()
{
	try
	{
        // robotValues( <radius, 0.25>, <radiusMargin, 0.05>, ... )
        std::vector< std::pair<std::string,std::string> > robotValues;
        environmentCommon::readYAML("robot", robotValues);

        // Find <radius, 0.25> in robotValues using key "radius"
        auto radiusPair = std::find_if( robotValues.begin(), robotValues.end(), KeyEquals("radius") ); 
        _radius = std::stof( radiusPair->second );

        // Find <radiusMargin, 0.xx> in robotValues using key "radiusMargin"
        auto radiusMarginPair = std::find_if( robotValues.begin(), robotValues.end(), KeyEquals("radiusMargin") ); 
        _radiusMargin = std::stof( radiusMarginPair->second );


	} catch (exception &e)
	{
    	printf("Invalid YAML format!\n");
    	TRACE("Invalid YAML format for reading environment field inputs");
		throw e;
	}
}
