 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

