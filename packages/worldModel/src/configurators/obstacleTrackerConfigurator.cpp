 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleTrackerConfigurator.cpp
 *
 *  Created on: Nov 20, 2016
 *      Author: Jan Feitsma
 */

#include "int/configurators/obstacleTrackerConfigurator.hpp"

#include <boost/lexical_cast.hpp>
#include <string>

#include "FalconsCommon.h"
#include "cDiagnosticsEvents.hpp"

void obstacleTrackerConfigurator::reset()
{
	// solver settings - object tracker
	_dataFloat[obstacleTrackerConfiguratorFloats::extrapolationTimeout] = 1.0; // [s] when to stop extrapolating
	_dataFloat[obstacleTrackerConfiguratorFloats::trackerTimeout] = 1.0; // [s] when to discard measurements
	_dataFloat[obstacleTrackerConfiguratorFloats::trackerXYTolerance] = 0.5; // [m] measurement grouping criterion
	_dataFloat[obstacleTrackerConfiguratorFloats::filterXYownTolerance] = 0.5;
	_dataFloat[obstacleTrackerConfiguratorFloats::filterXYmemberTolerance] = 0.5;
	_dataFloat[obstacleTrackerConfiguratorFloats::discriminatorLimitX] = 6.3;
	_dataFloat[obstacleTrackerConfiguratorFloats::discriminatorLimitY] = 9.3;
	_dataFloat[obstacleTrackerConfiguratorFloats::outlierNSigma] = 3.0;
    _dataFloat[obstacleTrackerConfiguratorFloats::outlierIterFraction] = 0.1; 
    _dataFloat[obstacleTrackerConfiguratorFloats::speedMinSize] = 0.3; 
    _dataFloat[obstacleTrackerConfiguratorFloats::speedMaxSize] = 5.0; 
    _dataFloat[obstacleTrackerConfiguratorFloats::speedResidualThreshold] = 0.5; 
	_dataInteger[obstacleTrackerConfiguratorIntegers::outlierMaxIter] = 1;
	_dataInteger[obstacleTrackerConfiguratorIntegers::speedFitOrder] = 0;
}

obstacleTrackerConfigurator::obstacleTrackerConfigurator()
{
    reset();
    
	// enum2str
    _enum2strFloat[obstacleTrackerConfiguratorFloats::extrapolationTimeout] = "extrapolationTimeout";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::trackerTimeout] = "trackerTimeout";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::trackerXYTolerance] = "trackerXYTolerance";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::filterXYownTolerance] = "filterXYownTolerance";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::filterXYmemberTolerance] = "filterXYmemberTolerance";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::discriminatorLimitX] = "discriminatorLimitX";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::discriminatorLimitY] = "discriminatorLimitY";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::outlierNSigma] = "outlierNSigma";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::outlierIterFraction] = "outlierIterFraction";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::speedMinSize] = "speedMinSize";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::speedMaxSize] = "speedMaxSize";
    _enum2strFloat[obstacleTrackerConfiguratorFloats::speedResidualThreshold] = "speedResidualThreshold";
    _enum2strInteger[obstacleTrackerConfiguratorIntegers::outlierMaxIter] = "outlierMaxIter";
	_enum2strInteger[obstacleTrackerConfiguratorIntegers::speedFitOrder] = "speedFitOrder";
}

// TODO @Tim: remainder of this file is highly duplicate w.r.t. ballTrackerConfigurator... 
// can we factor out a base class? only issue is that the enums are specific --> templatize?

obstacleTrackerConfigurator::~obstacleTrackerConfigurator()
/*
 * Big foot claims he saw Chuck Norris.
 */
{

}

void obstacleTrackerConfigurator::set(const obstacleTrackerConfiguratorBool key, const bool value)
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

void obstacleTrackerConfigurator::set(const obstacleTrackerConfiguratorIntegers key, const int value)
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

void obstacleTrackerConfigurator::set(const obstacleTrackerConfiguratorFloats key, const float value)
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

bool obstacleTrackerConfigurator::set(const std::string key, const std::string value)
{
    bool isFound = false;

    for (auto it = _dataInteger.begin(); ((it != _dataInteger.end()) && !isFound); ++it)
    {
        if (_enum2strInteger[it->first] == key)
        {
            set(it->first, boost::lexical_cast<int>(value));
            isFound = true;
        }
    }
    for (auto it = _dataFloat.begin(); ((it != _dataFloat.end()) && !isFound); ++it)
    {
        if (_enum2strFloat[it->first] == key)
        {
            set(it->first, boost::lexical_cast<float>(value));
            isFound = true;
        }
    }
    for (auto it = _dataBool.begin(); ((it != _dataBool.end()) && !isFound); ++it)
    {
        if (_enum2strBool[it->first] == key)
        {
            set(it->first, boost::lexical_cast<bool>(value));
            isFound = true;
        }
    }

    return isFound;
}

bool obstacleTrackerConfigurator::get(const obstacleTrackerConfiguratorBool key) const
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

int obstacleTrackerConfigurator::get(const obstacleTrackerConfiguratorIntegers key) const
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

float obstacleTrackerConfigurator::get(const obstacleTrackerConfiguratorFloats key) const
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

std::string obstacleTrackerConfigurator::enum2str(const obstacleTrackerConfiguratorBool key) const
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

std::string obstacleTrackerConfigurator::enum2str(const obstacleTrackerConfiguratorFloats key) const
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

std::string obstacleTrackerConfigurator::enum2str(const obstacleTrackerConfiguratorIntegers key) const
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

void obstacleTrackerConfigurator::traceAll()
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

