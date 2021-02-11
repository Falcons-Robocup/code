// Copyright 2016-2020 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cEnvironmentBall.cpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Michel Koenen
 */

#include <ext/cEnvironmentCommon.hpp>
#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "ext/cEnvironmentBall.hpp"
using namespace std;


cEnvironmentBall::cEnvironmentBall()
{
	getConfig();
}

cEnvironmentBall::~cEnvironmentBall()
{

}

float cEnvironmentBall::getRadius()
{
	return _radius;
};

/*! read the YAML file(s) as part of constructing phase and fill the private class members containing the values
 *
 */
void cEnvironmentBall::getConfig()
{
	try
	{

        // ballValues( <radius, 0.15>, ... )
        std::vector< std::pair<std::string,std::string> > ballValues;
        environmentCommon::readYAML("ball", ballValues);

        // Find <radius, 0.15> in ballValues using key "radius"
        auto radiusPair = std::find_if( ballValues.begin(), ballValues.end(), KeyEquals("radius") ); 
        _radius = std::stof( radiusPair->second );

	} catch (exception &e)
	{
    	printf("Invalid YAML format!");
    	TRACE("Invalid YAML format for reading environment field inputs");
		throw e;
	}
}
