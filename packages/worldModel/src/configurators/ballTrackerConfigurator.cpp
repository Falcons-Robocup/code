 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballTrackerConfigurator.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#include "int/configurators/ballTrackerConfigurator.hpp"

#include <boost/lexical_cast.hpp>
#include <string>

#include "FalconsCommon.h"
#include "cDiagnostics.hpp"
#include "tracing.hpp"

void ballTrackerConfigurator::reset()
{
    // solver settings - core fit
    _dataFloat[ballTrackerConfiguratorFloats::solverCoreWeight] = 0.1; // [1] relative depth weights
    _dataBool[ballTrackerConfiguratorBool::solverCoreSpeed] = true; // [bool] fit with speed - on robot, we always try to fit with speed
    _dataFloat[ballTrackerConfiguratorFloats::solverCoreMinVdt] = 0.05; // [s] minimum measurement spread required for speed fitting, otherwise fallback to position-only
    _dataInteger[ballTrackerConfiguratorIntegers::solverCoreMinVmeas] = 3;     // [1] minimum number of measurements required for speed fitting, otherwise fallback to position-only

    // solver settings - bounce detection
    _dataFloat[ballTrackerConfiguratorFloats::solverBounceDt] = 0.1; // [s] bounce detection resolution
    _dataFloat[ballTrackerConfiguratorFloats::solverBounceAge] = 0.5; // [s] how far to look back in time in solver
    _dataInteger[ballTrackerConfiguratorIntegers::solverBounceMinMeasurements] = 5; // [1] minimum number of measurements per segment
    _dataFloat[ballTrackerConfiguratorFloats::solverMinDv] = 6.0; // [m/s] minimum speed change for bounce detection
    _dataFloat[ballTrackerConfiguratorFloats::groupingDt] = 0.033; // [s] grouping time tolerance for triangulation
    _dataFloat[ballTrackerConfiguratorFloats::outlierNSigma] = 3.0; // [1] spread threshold for outlier removal
    _dataInteger[ballTrackerConfiguratorIntegers::outlierMaxIter] = 1; // number of iterations (1 is minimum, no outlier removal)
    _dataFloat[ballTrackerConfiguratorFloats::outlierIterFraction] = 0.1; 
    _dataInteger[ballTrackerConfiguratorIntegers::measPerOrder] = 5; // [1] required number of FCS measurements per fit order

    // solver settings - data selection
    _dataBool[ballTrackerConfiguratorBool::useOwnHighVision] = false; // [bool] use own HighVision measurements in algorithm 
    _dataBool[ballTrackerConfiguratorBool::useFriendlyHighVision] = false; // [bool] use friendly HighVision measurements in algorithm 
    _dataBool[ballTrackerConfiguratorBool::shareHighVision] = false; // [bool] share own Highvision measurements with friends
    
    // solver settings - blacklisting
    _dataBool[ballTrackerConfiguratorBool::blackListDefault] = false; // [bool] accept a new tracker which has not yet been blacklisted
    _dataFloat[ballTrackerConfiguratorFloats::blackListThresholdZ] = 0.4; // [m] height threshold of ball measurement in FCS used in blacklisting
    _dataFloat[ballTrackerConfiguratorFloats::blackListFloatingDuration] = 1.0; // [s] time after which to blacklist a tracker
    _dataFloat[ballTrackerConfiguratorFloats::blackListGroundDuration] = 1.0; // [s] time after which to whitelist a tracker
    
    // solver settings - object tracker
    _dataFloat[ballTrackerConfiguratorFloats::solverTrackerTimeout] = 2.0; // [s] when to discard trackers
    _dataFloat[ballTrackerConfiguratorFloats::solverTrackerBuffer] = 1.0; // [s] when to discard measurements within tracker
    _dataFloat[ballTrackerConfiguratorFloats::solverTrackerConeTolerance] = 0.1; // [rad] measurement grouping criterion
    _dataInteger[ballTrackerConfiguratorIntegers::numberOfBallsWarningThreshold] = 1; // [1] warn in case more than this amount of balls are identified
    _dataFloat[ballTrackerConfiguratorFloats::confidenceGoodLimit] = 0.5; // [1] all balls with a confidence higher than this threshold are accepted
    _dataFloat[ballTrackerConfiguratorFloats::confidenceMaybeLimit] = 0.2; // [1] if no good balls are seen, then this limit is used
    _dataInteger[ballTrackerConfiguratorIntegers::maxMaybeBalls] = 1; // [1] amount of 'maybe' balls to accept
        
    // solver settings - confidence heuristics
    _dataInteger[ballTrackerConfiguratorIntegers::confidenceNumCams] = 3; // [1] required number of involved cameras for full confidence
    _dataInteger[ballTrackerConfiguratorIntegers::confidenceMeasLim] = 20; // [1] required number of good (i.e. non-outlier) time-clustered measurements for full confidence
    _dataBool[ballTrackerConfiguratorBool::confidenceOmniPref] = true; // [bool] omnivision required for full confidence
    _dataFloat[ballTrackerConfiguratorFloats::confidenceFreshLim] = 0.0; // [s] required freshness of tracker for full confidence, see also tracker timeout
    _dataFloat[ballTrackerConfiguratorFloats::confidenceAgeLim] = 5.0; // [s] required age of tracker for full confidence (if tracker starts at t=1, tcurr=4, then age=3)
    _dataFloat[ballTrackerConfiguratorFloats::confidenceZLim1] = 1.0; // [m] threshold for fitted z result - we consider values lower than this threshold good
    _dataFloat[ballTrackerConfiguratorFloats::confidenceZLim2] = 5.0; // [m] threshold for fitted z result - we consider values higher than this threshold bad
    _dataFloat[ballTrackerConfiguratorFloats::confidenceVLim1] = 2.0; // [m/s] speed (vx,vy,vz) threshold - we consider values lower than this threshold good
    _dataFloat[ballTrackerConfiguratorFloats::confidenceVLim2] = 20.0; // [m/s] speed (vx,vy,vz) threshold - we consider values higher than this threshold bad
    _dataFloat[ballTrackerConfiguratorFloats::confidenceFitLim1] = 0.20; // [1] numerical fit residue threshold - we consider values lower than this threshold good
    _dataFloat[ballTrackerConfiguratorFloats::confidenceFitLim2] = 0.60; // [1] numerical fit residue threshold - we consider values higher than this threshold bad
    _dataFloat[ballTrackerConfiguratorFloats::friendlyMeasurementsDistance] = 0.0; // [m] 
}

ballTrackerConfigurator::ballTrackerConfigurator()
{
    reset();
    
    // enum2str
    _enum2strBool[ballTrackerConfiguratorBool::useOwnHighVision] = "useOwnHighVision";
    _enum2strBool[ballTrackerConfiguratorBool::useFriendlyHighVision] = "useFriendlyHighVision";
    _enum2strBool[ballTrackerConfiguratorBool::shareHighVision] = "shareHighVision";
    _enum2strFloat[ballTrackerConfiguratorFloats::solverCoreWeight] = "solverCoreWeight";
    _enum2strBool[ballTrackerConfiguratorBool::solverCoreSpeed] = "solverCoreSpeed";
    _enum2strFloat[ballTrackerConfiguratorFloats::solverCoreMinVdt] = "solverCoreMinVdt";
    _enum2strInteger[ballTrackerConfiguratorIntegers::solverCoreMinVmeas] = "solverCoreMinVmeas";
    _enum2strFloat[ballTrackerConfiguratorFloats::solverBounceDt] = "solverBounceDt";
    _enum2strFloat[ballTrackerConfiguratorFloats::solverBounceAge] = "solverBounceAge";
    _enum2strInteger[ballTrackerConfiguratorIntegers::solverBounceMinMeasurements] = "solverBounceMinMeasurements";
    _enum2strFloat[ballTrackerConfiguratorFloats::solverMinDv] = "solverMinDv";
    _enum2strFloat[ballTrackerConfiguratorFloats::solverTrackerTimeout] = "solverTrackerTimeout";
    _enum2strFloat[ballTrackerConfiguratorFloats::solverTrackerBuffer] = "solverTrackerBuffer";    
    _enum2strFloat[ballTrackerConfiguratorFloats::solverTrackerConeTolerance] = "solverTrackerConeTolerance";
    _enum2strInteger[ballTrackerConfiguratorIntegers::numberOfBallsWarningThreshold] = "numberOfBallsWarningThreshold";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceGoodLimit] = "confidenceGoodLimit";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceMaybeLimit] = "confidenceMaybeLimit";
    _enum2strInteger[ballTrackerConfiguratorIntegers::maxMaybeBalls] = "maxMaybeBalls";
    _enum2strInteger[ballTrackerConfiguratorIntegers::confidenceMeasLim] = "confidenceMeasLim";
    _enum2strInteger[ballTrackerConfiguratorIntegers::confidenceNumCams] = "confidenceNumCams";
    _enum2strBool[ballTrackerConfiguratorBool::confidenceOmniPref] = "confidenceOmniPref";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceFreshLim] = "confidenceFreshLim";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceAgeLim] = "confidenceAgeLim";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceFitLim1] = "confidenceFitLim1";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceFitLim2] = "confidenceFitLim2";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceVLim1] = "confidenceVLim1";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceVLim2] = "confidenceVLim2";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceZLim1] = "confidenceZLim1";
    _enum2strFloat[ballTrackerConfiguratorFloats::confidenceZLim2] = "confidenceZLim2";
    _enum2strFloat[ballTrackerConfiguratorFloats::groupingDt] = "groupingDt";
    _enum2strFloat[ballTrackerConfiguratorFloats::outlierNSigma] = "outlierNSigma";
    _enum2strFloat[ballTrackerConfiguratorFloats::outlierIterFraction] = "outlierIterFraction";
    _enum2strInteger[ballTrackerConfiguratorIntegers::outlierMaxIter] = "outlierMaxIter";
    _enum2strInteger[ballTrackerConfiguratorIntegers::measPerOrder] = "measPerOrder";
    _enum2strFloat[ballTrackerConfiguratorFloats::friendlyMeasurementsDistance] = "friendlyMeasurementsDistance";
    _enum2strFloat[ballTrackerConfiguratorFloats::blackListThresholdZ] = "blackListThresholdZ";
    _enum2strFloat[ballTrackerConfiguratorFloats::blackListFloatingDuration] = "blackListFloatingDuration";
    _enum2strFloat[ballTrackerConfiguratorFloats::blackListGroundDuration] = "blackListGroundDuration";
    _enum2strBool[ballTrackerConfiguratorBool::blackListDefault] = "blackListDefault";
}

ballTrackerConfigurator::~ballTrackerConfigurator()
/*
 * Big foot claims he saw Chuck Norris.
 */
{

}

void ballTrackerConfigurator::set(const ballTrackerConfiguratorBool key, const bool value)
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

void ballTrackerConfigurator::set(const ballTrackerConfiguratorIntegers key, const int value)
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

void ballTrackerConfigurator::set(const ballTrackerConfiguratorFloats key, const float value)
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

bool ballTrackerConfigurator::set(const std::string key, const std::string value)
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

bool ballTrackerConfigurator::get(const ballTrackerConfiguratorBool key) const
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

int ballTrackerConfigurator::get(const ballTrackerConfiguratorIntegers key) const
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

float ballTrackerConfigurator::get(const ballTrackerConfiguratorFloats key) const
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

std::string ballTrackerConfigurator::enum2str(const ballTrackerConfiguratorBool key) const
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

std::string ballTrackerConfigurator::enum2str(const ballTrackerConfiguratorFloats key) const
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

std::string ballTrackerConfigurator::enum2str(const ballTrackerConfiguratorIntegers key) const
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

void ballTrackerConfigurator::traceAll()
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

    // trace it - TODO do this only once, at start of wm initialization, after yaml load, and upon re-configure
    //tprintf("%s", s.c_str());
}

