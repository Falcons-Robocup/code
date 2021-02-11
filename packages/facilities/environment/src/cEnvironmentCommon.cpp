// Copyright 2016-2020 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cEnvironmentCommon.cpp
 *
 *  Created on: Jan 3, 2016
 *      Author: Michel Koenen
 */

#include <yaml-cpp/yaml.h>

#include <ext/cEnvironmentCommon.hpp>
#include "falconsCommon.hpp"
#include <stdlib.h>      // for getenv
#include "tracing.hpp"

#include <fstream>
#include <iostream>


//Loads the entire file, newlines and all, and returns its contents as a string.
//Returns an empty string if the file doesn't exist.

/*! Load the specified file including newlines and returns its contents as a string
 *
 * @param[in] filename  please specify the full path to the file as well, otherwise it will be relative with high risk of not finding the file during runtime
 * @param[out] successful   will indicate if the file could be read
 * @return  file contents as string
 */
std::string environmentCommon::LoadFileAsString(const std::string &filename, bool *successful = nullptr)
{
	if(successful) *successful = false;

	//Open the file for reading.
	std::ifstream file(filename.c_str());
	if(!file)
	{
		std::string error_string=" { \"ERROR\":  \"could not find or open file\" }";
		// File not found or other file issue, dont fail here, return message anyway
		return error_string;
	}

	//Read the file into the stringStream.
	std::ostringstream stringStream;
	stringStream << file.rdbuf();

	if(successful) *successful = true;

	//Convert the stringStream into a regular string.
	return stringStream.str();
}


bool environmentCommon::readYAML(std::string root, std::vector< std::pair<std::string,std::string> > &values)
{
	try
	{
		std::string yamlFilePath = pathToConfig() + "/cEnvironment.yaml";

		/*file contents something like:
			{
				"field"	:
				{
					"width":		12.0,
					"length":		18.0,
					"goalPostOffset":	1.0,
					"goalAreaOffset":	0.75,
					"goalDepth":		0.6,
					"penaltyAreaOffset":	2.25,
					"penaltySpotOffset":	3.0,
					"centerCircleRadius":	2.0,
					"safetyBoundaryOffset":	1.50,
					"lineThickness":	0.125
				},
				"ball"	:
				{
					"radius":		0.15
				},
				"robot"	:
				{
					"radius":		0.25
				}

			}

		*/


        // Load the YAML and get all entries.
        YAML::Node yamlNode = YAML::LoadFile(yamlFilePath);
        YAML::Node rootNode = yamlNode[root];

        YAML::const_iterator it;
        for(it = rootNode.begin(); it != rootNode.end(); ++it)
        {
            // Add each yaml entry to values
            values.push_back( std::make_pair( it->first.as<std::string>(), it->second.as<std::string>() ) );
        }
		return true;

	} catch(std::exception& e)
	{
		printf("Invalid YAML format!\n");
		TRACE("Invalid YAML format for reading environment field inputs");
		throw e;
	}
    return false;
}

