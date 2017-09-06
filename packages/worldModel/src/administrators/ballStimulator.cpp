 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballStimulator.cpp
 *
 *  Created on: Sep 10, 2016
 *      Author: Jan Feitsma
 */

#include "int/administrators/ballStimulator.hpp"
#include "int/configurators/ballTrackerConfigurator.hpp"

#include "cDiagnosticsEvents.hpp"

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <sstream>

ballStimulator::ballStimulator(std::string const &filename)
{
	_uIDCounter = 0;
    ballTrackerConfigurator::getInstance().reset();
    ballTracker::reset();
    loadInput(filename);
    run();
}

ballStimulator::~ballStimulator()
{
}

bool ballStimulator::compare()
{
    TRACE("comparing results (file-based)");
    // trick: write to temp files, then compare contents
    std::string tmpFilenameExpected = tmpFileName();
    writeOutput(tmpFilenameExpected, 0);
    std::string tmpFilenameGot = tmpFileName();
    writeOutput(tmpFilenameGot, 1);
    bool equal = fileContentEqual(tmpFilenameExpected, tmpFilenameGot);
    if (equal)
    {
        remove(tmpFilenameExpected.c_str());
        remove(tmpFilenameGot.c_str());
    }
    else
    {
        // this typically happens during development, give developer the opportunity
        // to replace the file with expected results
        TRACE("NOTE: file content comparison failed, leaving tmp filenames for further inspection (got=%s, expected=%s)", tmpFilenameGot.c_str(), tmpFilenameExpected.c_str());
    }
    return equal;
}

float zeroFix(float v)
{
    // because we do file-based comparisons in test suite, we often end up comparing -0.0 with 0.0
    // to avoid this, we could either make the test suite robust, or prevent writing -0.0 in the first place
    // latter option is chosen
    if ((v < 0) && (v > -0.0005))
    {
        v = 0;
    }
    return v;
}

void ballStimulator::writeOutput(std::string const &filename, int what)
{
    // choose which solution data to write
    auto solutions = _solutionsGot;
    if (what == 0)
    {
        solutions = _solutionsExpected;
    }
    // write to file
    FILE *fp = fopen(filename.c_str(), "w");
    int numLines = 0;
    for (auto it = solutions.begin(); it != solutions.end(); ++it)
    {
        std::vector<ballClass_t> balls = it->second;
        double t = it->first;
        for (size_t iball = 0; iball < balls.size(); ++iball)
        {
            ballClass_t ball = balls[iball];
            // keep format consistent with fbt_sol_2str.m
            fprintf(fp, "%16.6f  %2d  %5d  %4.2f  %7.3f  %7.3f  %7.3f  %7.3f  %7.3f  %7.3f\n",
                    t, (int)balls.size(), (int)ball.getId(), ball.getConfidence(), 
                    zeroFix(ball.getX()), zeroFix(ball.getY()), zeroFix(ball.getZ()),
                    zeroFix(ball.getVX()), zeroFix(ball.getVY()), zeroFix(ball.getVZ()));
            numLines++;
        }
    }
    fclose(fp);
    TRACE("%d lines written to %s", numLines, filename.c_str());
}

void ballStimulator::run()
{
    TRACE("starting stimulation");
    ballTrackerConfigurator::getInstance().traceAll();
    // for each solver tick: feed measurements and query the solver
    size_t imeas = 0;
    for (size_t itick = 0; itick < _solverTicks.size(); ++itick)
    {
        double t = _solverTicks[itick];
        TRACE("tick=%16.6f", t);
        // feed measurements
        int count = 0;
        while ((imeas < _measurements.size()) && (_measurements[imeas].getTimestamp() <= t))
        {
            _discriminator.addMeasurement(_measurements[imeas]);
            imeas++;
            count++;
        }
        TRACE("added %d measurements", count);
        // query solver
        _discriminator.performCalculation(t);
        // store result
        _solutionsGot[t] = _discriminator.getBalls();
        TRACE("got %d ball results", (int)_solutionsGot[t].size());
    }
    TRACE("finished stimulation");
}

void ballStimulator::setConfig(std::vector<std::string> const &kvPairs)
{
    for (size_t it = 0; it < kvPairs.size(); ++it)
    {
        std::vector<std::string> keyAndValue;
        boost::split(keyAndValue, kvPairs[it], boost::is_any_of("="));
        if (keyAndValue.size() == 2)
        {
            std::string key = keyAndValue[0];
            std::string valueAsString = keyAndValue[1];
            ballTrackerConfigurator::getInstance().set(key, valueAsString);
        }
    }
}
        
void ballStimulator::addMeasurement(std::vector<std::string> const &values)
{
    ballMeasurementType m;
    m.setID(uniqueWorldModelID(boost::lexical_cast<int>(values[0]), _uIDCounter++));
    //m._cameraType = INVALID; // unused for now 
    m.setTimestamp(boost::lexical_cast<double>(values[2]));
    m.setCameraOffset(boost::lexical_cast<float>(values[3]),
                      boost::lexical_cast<float>(values[4]),
                      boost::lexical_cast<float>(values[5]),
                      boost::lexical_cast<float>(values[6]));
    m.setSphericalCoords(boost::lexical_cast<float>(values[7]),
                         boost::lexical_cast<float>(values[8]),
                         boost::lexical_cast<float>(values[9]));
    m.setConfidence(boost::lexical_cast<float>(values[10]));
    _measurements.push_back(m);
}

void ballStimulator::addSolution(std::vector<std::string> const &values)
{
    ballClass_t s;
    s.setTimestamp(boost::lexical_cast<double>(values[0]));
    size_t numBalls = boost::lexical_cast<int>(values[1]);
    if (numBalls)
    {
        s.setId(boost::lexical_cast<int>(values[2])); // tracker id is used in diagnostics / visualizer, to draw balls uniquely and be able to reposition them
        s.setConfidence(boost::lexical_cast<float>(values[3]));
        s.setCoordinates(boost::lexical_cast<float>(values[4]),
                         boost::lexical_cast<float>(values[5]),
                         boost::lexical_cast<float>(values[6]));
        s.setVelocities(boost::lexical_cast<float>(values[7]),
                        boost::lexical_cast<float>(values[8]),
                        boost::lexical_cast<float>(values[9]));
        _solutionsExpected[s.getTimestamp()].push_back(s);
        if (!_solverTicks.size() || (_solverTicks.back() != s.getTimestamp()))
        {
        	_solverTicks.push_back(s.getTimestamp());
    	}
    }
    else
    {
        // no balls at this tick, but store it anyway
    	_solverTicks.push_back(s.getTimestamp());
    }
}

void ballStimulator::loadInput(std::string const &filename)
{
    TRACE("loading file %s", filename.c_str());
    std::ifstream infile(filename.c_str());
    assert(infile); // file is expected to exist...
    std::string line;
    _solverTicks.clear();
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
            if (words[0] == "config")
            {
                words.erase(words.begin());
                setConfig(words);
            }
            else
            {
                if (words.size() == 11)
                {
                    addMeasurement(words);
                }
                else if (words.size() == 10)
                {
                    addSolution(words);
                }
            }
        }
    }
    // trace result counts
    TRACE("done - loaded %d measurements, %d ticks and %d solutions", _measurements.size(), _solverTicks.size(), _solutionsExpected.size());
}

bool ballStimulator::fileContentEqual(std::string const &filename1, std::string const &filename2)
{
    std::string cmd = "diff -q " + filename1 + " " + filename2;
    int r = system(cmd.c_str());
    TRACE("r=%d cmd='%s'", r, cmd.c_str());
    return (r == 0);
}

std::string ballStimulator::tmpFileName()
{
    static int counter = 0;
    return "/var/tmp/ballStimTmp" + boost::lexical_cast<std::string>(counter++);
}

