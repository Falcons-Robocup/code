 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * localizationConfigurator.cpp
 *
 *  Created on: Dec 11, 2016
 *      Author: Jan Feitsma
 */

#include "int/configurators/localizationConfigurator.hpp"

#include <boost/lexical_cast.hpp>
#include <string>

#include "FalconsCommon.h"
#include "cDiagnostics.hpp"
#include "tracing.hpp"

void localizationConfigurator::reset()
{
    // values: see .yaml
    // description: see .cfg
    // NOTE: unless overridden, these are the default values for test suite
    _dataFloat[localizationConfiguratorFloats::errorRatioRadianToMeter] = 0.5;
    _dataFloat[localizationConfiguratorFloats::visionOwnWeightFactor] = 0.2;
    _dataFloat[localizationConfiguratorFloats::trackerScoreAcceptanceThreshold] = 1.5;
    _dataFloat[localizationConfiguratorFloats::trackerTimeout] = 2.0;
    _dataFloat[localizationConfiguratorFloats::scoreActivityScale] = 5.0;
    _dataFloat[localizationConfiguratorFloats::scoreAgeScale] = 20.0;
    _dataFloat[localizationConfiguratorFloats::scoreFreshScale] = 0.5;
    _dataFloat[localizationConfiguratorFloats::settlingTime] = 3.0;
    _dataFloat[localizationConfiguratorFloats::minimumConfidence] = 0.9;
    _dataFloat[localizationConfiguratorFloats::speedLimitXY] = 5.0;
    _dataFloat[localizationConfiguratorFloats::speedLimitPhi] = 6.0;
    _dataFloat[localizationConfiguratorFloats::positionLimitX] = 7.5;
    _dataFloat[localizationConfiguratorFloats::positionLimitY] = 10.5;
    _dataInteger[localizationConfiguratorIntegers::visionStabilityLength] = 0;
}

localizationConfigurator::localizationConfigurator()
{
    reset();
    
	// enum2str
    _enum2strFloat[localizationConfiguratorFloats::errorRatioRadianToMeter] = "errorRatioRadianToMeter";
    _enum2strFloat[localizationConfiguratorFloats::visionOwnWeightFactor] = "visionOwnWeightFactor";
    _enum2strFloat[localizationConfiguratorFloats::trackerScoreAcceptanceThreshold] = "trackerScoreAcceptanceThreshold";
    _enum2strFloat[localizationConfiguratorFloats::trackerTimeout] = "trackerTimeout";
    _enum2strFloat[localizationConfiguratorFloats::scoreActivityScale] = "scoreActivityScale";
    _enum2strFloat[localizationConfiguratorFloats::scoreAgeScale] = "scoreAgeScale";
    _enum2strFloat[localizationConfiguratorFloats::scoreFreshScale] = "scoreFreshScale";
    _enum2strFloat[localizationConfiguratorFloats::settlingTime] = "settlingTime";
    _enum2strFloat[localizationConfiguratorFloats::minimumConfidence] = "minimumConfidence";
    _enum2strFloat[localizationConfiguratorFloats::speedLimitXY] = "speedLimitXY";
    _enum2strFloat[localizationConfiguratorFloats::speedLimitPhi] = "speedLimitPhi";
    _enum2strFloat[localizationConfiguratorFloats::positionLimitX] = "positionLimitX";
    _enum2strFloat[localizationConfiguratorFloats::positionLimitY] = "positionLimitY";
    _enum2strInteger[localizationConfiguratorIntegers::visionStabilityLength] = "visionStabilityLength";
    
}

localizationConfigurator::~localizationConfigurator()
/*
 * Big foot claims he saw Chuck Norris.
 */
{

}

// TODO code below is highly duplicate w.r.t. ball- and obstacleConfigurator, consider base class or template

void localizationConfigurator::set(const localizationConfiguratorBool key, const bool value)
{
	try
	{
		_dataBool.at(key) = value;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void localizationConfigurator::set(const localizationConfiguratorIntegers key, const int value)
{
	try
	{
		_dataInteger.at(key) = value;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void localizationConfigurator::set(const localizationConfiguratorFloats key, const float value)
{
	try
	{
		_dataFloat.at(key) = value;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool localizationConfigurator::set(const std::string key, const std::string value)
{
    for (auto it = _dataInteger.begin(); it != _dataInteger.end(); ++it)
    {
        if (_enum2strInteger[it->first] == key)
        {
            set(it->first, boost::lexical_cast<int>(value));
            return true;
        }
    }
    for (auto it = _dataFloat.begin(); it != _dataFloat.end(); ++it)
    {
        if (_enum2strFloat[it->first] == key)
        {
            set(it->first, boost::lexical_cast<float>(value));
            return true;
        }
    }
    for (auto it = _dataBool.begin(); it != _dataBool.end(); ++it)
    {
        if (_enum2strBool[it->first] == key)
        {
            set(it->first, boost::lexical_cast<bool>(value));
            return true;
        }
    }
    return false;
}

bool localizationConfigurator::get(const localizationConfiguratorBool key) const
{
	try
	{
		return _dataBool.at(key);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

int localizationConfigurator::get(const localizationConfiguratorIntegers key) const
{
	try
	{
		return _dataInteger.at(key);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

float localizationConfigurator::get(const localizationConfiguratorFloats key) const
{
	try
	{
		return _dataFloat.at(key);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

std::string localizationConfigurator::enum2str(const localizationConfiguratorBool key) const
{
	try
	{
		return _enum2strBool.at(key);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

std::string localizationConfigurator::enum2str(const localizationConfiguratorFloats key) const
{
	try
	{
		return _enum2strFloat.at(key);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

std::string localizationConfigurator::enum2str(const localizationConfiguratorIntegers key) const
{
	try
	{
		return _enum2strInteger.at(key);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void localizationConfigurator::traceAll()
{
    // construct a string
    std::string s;
    try
    {
        for (auto it = _dataBool.begin(); it != _dataBool.end(); ++it)
        {
            assert(_enum2strBool.count(it->first));
            s += _enum2strBool[it->first];
            s += "=";
        	if(it->second)
        	{
        		s += "True";
        	}
        	else
        	{
        		s += "False";
        	}
    	    s += " ";
	    }
        for (auto it = _dataFloat.begin(); it != _dataFloat.end(); ++it)
        {
            assert(_enum2strFloat.count(it->first));
            s += _enum2strFloat[it->first];
            s += "=";
            s += boost::lexical_cast<std::string>(it->second);
    	    s += " ";
	    }
        for (auto it = _dataInteger.begin(); it != _dataInteger.end(); ++it)
        {
            assert(_enum2strInteger.count(it->first));
            s += _enum2strInteger[it->first];
            s += "=";
            s += boost::lexical_cast<std::string>(it->second);
    	    s += " ";
	    }
    }
    catch (std::exception &e)
    {
		s += "???";
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

    // trace it
    TRACE("%s", s.c_str());
}

