// Copyright 2016-2020 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
