 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * localizationStimulator.cpp
 *
 *  Created on: Jul 8, 2017
 *      Author: Jan Feitsma
 */

#include "int/administrators/localizationStimulator.hpp"
#include "int/configurators/localizationConfigurator.hpp"

#include "FalconsCommon.h"
#include <boost/algorithm/string.hpp>
#include <fstream>


localizationStimulator::localizationStimulator()
{
}

localizationStimulator::~localizationStimulator()
{
}

void localizationStimulator::load(std::string filename)
{
    TRACE("loading file %s", filename.c_str());
    std::ifstream infile(filename.c_str());
    assert(infile); // file is expected to exist...
    std::string line;
    _input.clear();
    _expectedResult.clear();
    localizationConfigurator::getInstance().reset();
    std::pair<double, std::pair<inputType, boost::any>> elem;
    double t;
    while (std::getline(infile, line))
    {
        // trim
        boost::trim_if(line, boost::is_any_of("\t "));
        // ignore comment line
        if (line.size())
        {
            if (line[0] == '#')
            {
                continue;
            }
        }
        // split words
        std::vector<std::string> words;
        boost::split(words, line, boost::is_any_of("\t \n"), boost::token_compress_on);
        if (words.size())
        {
            if (words.at(0) == "CONFIGURE")
            {
                localizationConfigurator::getInstance().set(words.at(1), words.at(2));
            } 
            else if (words.at(0) == "INITIALIZE")
            {
                t = boost::lexical_cast<double>(words.at(1));
                elem.first = t;
                elem.second.first = inputType::INITIALIZE;
                elem.second.second = 0;
                _input.insert(elem);
            }
            else if (words.at(0) == "CALCULATE")
            {
                t = boost::lexical_cast<double>(words.at(1));
                elem.first = t;
                elem.second.first = inputType::CALCULATE;
                elem.second.second = 0;
                _input.insert(elem);
            }
            else if ((words.at(0) == "ENCODER_RCS") || (words.at(0) == "ENCODER_FCS"))
            {
                t = boost::lexical_cast<double>(words.at(1));
                elem.first = t;
                elem.second.first = inputType::ENCODER;
                robotDisplacementClass_t r;
                r.setDisplacementSource(displacementType::MOTORS);
                if (words.at(0) == "ENCODER_RCS")
                {
                    r.setCoordinateType(coordinateType::ROBOT_COORDS);
                }
                else
                {
                    r.setCoordinateType(coordinateType::FIELD_COORDS);
                }
                r.setTimestamp(boost::lexical_cast<double>(words.at(2)));
                r.setDeltaPosition(boost::lexical_cast<double>(words.at(3)), boost::lexical_cast<double>(words.at(4)), boost::lexical_cast<double>(words.at(5)));
                r.setDeltaVelocity(boost::lexical_cast<double>(words.at(6)), boost::lexical_cast<double>(words.at(7)), boost::lexical_cast<double>(words.at(8)));
                elem.second.second = r;
                _input.insert(elem);
            }
            else if (words.at(0) == "VISION")
            {
                t = boost::lexical_cast<double>(words.at(1));
                elem.first = t;
                elem.second.first = inputType::VISION;
                robotMeasurementClass_t r;
                r.setTimestamp(boost::lexical_cast<double>(words.at(2)));
                r.setPosition(boost::lexical_cast<double>(words.at(3)), boost::lexical_cast<double>(words.at(4)), boost::lexical_cast<double>(words.at(5)));
                r.setConfidence(boost::lexical_cast<double>(words.at(6)));
                elem.second.second = r;
                _input.insert(elem);
            }
            else if (words.at(0) == "RESULT")
            {
                t = boost::lexical_cast<double>(words.at(1));
                robotClass_t r;
                r.setTimestamp(t);
                bool valid = boost::lexical_cast<double>(words.at(2));
                r = robotClass_t();
                r.setCoordinates(boost::lexical_cast<double>(words.at(3)), boost::lexical_cast<double>(words.at(4)), boost::lexical_cast<double>(words.at(5)));
                r.setVelocities(boost::lexical_cast<double>(words.at(6)), boost::lexical_cast<double>(words.at(7)), boost::lexical_cast<double>(words.at(8)));
                _expectedResult[t].first = valid;
                _expectedResult[t].second = r;
            }
            else if (words.at(0).size())
            {
                throw std::runtime_error("parse error: invalid line, starting with " + words.at(0));
            }
        }
    }
    // trace result counts
    TRACE("done - loaded %d inputs and %d outputs", _input.size(), _expectedResult.size());
}

void localizationStimulator::run()
{
    _calculatedResult.clear();
    localizationConfigurator::getInstance().traceAll();
    for (auto it = _input.begin(); it != _input.end(); ++it)
    {
        double t = it->first;
        switch (it->second.first)
        {
            case inputType::ENCODER:
                _localizationAlgorithm.addDisplacement(boost::any_cast<robotDisplacementClass_t>(it->second.second));
                break;
            case inputType::VISION:
                _localizationAlgorithm.addVisionMeasurement(boost::any_cast<robotMeasurementClass_t>(it->second.second));
                break;
            case inputType::INITIALIZE:
                _localizationAlgorithm.triggerInitialization(t);
                break;
            case inputType::CALCULATE:
                _localizationAlgorithm.calculatePositionAndVelocity(t);
                // and store
                _calculatedResult[t].first = _localizationAlgorithm.isValid();
                _calculatedResult[t].second = _localizationAlgorithm.getRobotPositionAndVelocity();
                break;
        }
    }
}

bool localizationStimulator::verify(bool exact)
{
    bool result = true;
    // if exact, then the result must exactly match
    // otherwise the calculated result may contain more entries than expected result
    if (exact && (_calculatedResult.size() != _expectedResult.size()))
    {
        TRACE("result size mismatch: _calculatedResult.size()=%d _expectedResult.size()=%d", _calculatedResult.size(), _expectedResult.size());
        result = false;
    }
    // check if every element of _expectedResult is found in _calculatedResult
    float TOL = 1e-2;
    float delta = 0, got = 0, expected = 0;
    for (auto it = _expectedResult.begin(); it != _expectedResult.end(); ++it)
    {
        double t = it->first;
        if (!_calculatedResult.count(t))
        {
            TRACE("expected element not found for t=%16.6f", t);
            result = false;
        }
        // check valid flag
        if (it->second.first != _calculatedResult[t].first)
        {
            TRACE("valid flag mismatch at t=%16.6f: got=%d expected=%d", t, _calculatedResult[t].first, it->second.first);
            result = false;
        }
        // check X
        expected = it->second.second.getX(); 
        got = _calculatedResult[t].second.getX();
        delta = expected - got;
        if (fabs(delta) > TOL) 
        {
            TRACE("X mismatch at t=%16.6f: got=%6.2f expected=%6.2f", t, got, expected);
            result = false;
        }
        // check Y
        expected = it->second.second.getY(); 
        got = _calculatedResult[t].second.getY();
        delta = expected - got;
        if (fabs(delta) > TOL) 
        {
            TRACE("Y mismatch at t=%16.6f: got=%6.2f expected=%6.2f", t, got, expected);
            result = false;
        }
        // check Theta
        expected = it->second.second.getTheta(); 
        got = _calculatedResult[t].second.getTheta();
        delta = expected - got;
        if (fabs(delta) > TOL) 
        {
            TRACE("Theta mismatch at t=%16.6f: got=%6.2f expected=%6.2f", t, got, expected);
            result = false;
        }
        // check VX
        expected = it->second.second.getVX(); 
        got = _calculatedResult[t].second.getVX();
        delta = expected - got;
        if (fabs(delta) > TOL) 
        {
            TRACE("VX mismatch at t=%16.6f: got=%6.2f expected=%6.2f", t, got, expected);
            result = false;
        }
        // check VY
        expected = it->second.second.getVY(); 
        got = _calculatedResult[t].second.getVY();
        delta = expected - got;
        if (fabs(delta) > TOL) 
        {
            TRACE("VY mismatch at t=%16.6f: got=%6.2f expected=%6.2f", t, got, expected);
            result = false;
        }
        // check VTheta
        expected = it->second.second.getVTheta(); 
        got = _calculatedResult[t].second.getVTheta();
        delta = expected - got;
        if (fabs(delta) > TOL) 
        {
            TRACE("VTheta mismatch at t=%16.6f: got=%6.2f expected=%6.2f", t, got, expected);
            result = false;
        }
    }
    return result;
}

