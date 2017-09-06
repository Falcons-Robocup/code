 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * objectTracker.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: Jan Feitsma
 */

#include "int/algorithms/objectTracking.hpp"
#include "int/algorithms/objectCoreFit.hpp"

objectTracker::objectTracker()
{
	_residual = 0.0;
    _maxSpread = 0.0;
    _avgGroupSize = 0.0;
}

objectTracker::~objectTracker()
{
    // Chuck Norris doesn't read books. He stares them down until he gets the information he wants.
}

objectResultType objectTracker::getResult() const
{
    return _result;
}

float objectTracker::getFitResidual() const
{
    return _residual;
}

int objectTracker::getNumRemoved() const
{
    return _numRemoved;
}

float objectTracker::getTimeSpread() const
{
    return _maxSpread;
}

void objectTracker::setConfig(objectFitConfig cfg)
{
    _objectFitCfg = cfg;
}

void objectTracker::groupMeasurements()
{
    assert(_measurementsPtr != NULL);
    assert(_measurementsPtr->size() > 0);
    // initialize outputs
    _groupedTime.clear();
    _groupedMeasurements.clear();
    double currT = 0.0;
    double lastT = -1;
    double prevT = -1;
    double minT = _measurementsPtr->begin()->getObjectMeasurement().getTimestamp();
    double sumT = 0.0;
    _maxSpread = 0.0;
    std::vector<objectMeasurementCache> currentGroup;
    // perform grouping, use a grouping tolerance 'groupingDt'
    for (auto it = _measurementsPtr->begin(); it != _measurementsPtr->end(); ++it)
    {
        currT = it->getObjectMeasurement().getTimestamp();
        assert(prevT <= currT); // require data to be sorted in time
        if (currT - lastT > _objectFitCfg.groupingDt)
        {
            // close previous group
            if (currentGroup.size())
            {
                _groupedMeasurements.push_back(currentGroup);
                lastT = sumT / currentGroup.size(); // average time for this group
                _groupedTime.push_back(lastT);
                _maxSpread = std::max(_maxSpread, (float)(prevT - minT));
            }
            // start new group
            currentGroup.clear();
            sumT = 0.0;
            minT = currT;
        }
        // add to current group
        currentGroup.push_back(*it);
        sumT += currT;
        prevT = currT;
    }
    // close last group
    _groupedMeasurements.push_back(currentGroup);
    lastT = sumT / currentGroup.size();
    _groupedTime.push_back(lastT);
    _maxSpread = std::max(_maxSpread, (float)(prevT - minT));
    // for diagnostics, also store average group size
    _avgGroupSize = (float)_measurementsPtr->size() / _groupedMeasurements.size();
}

Vector3D objectTracker::triangulate(std::vector<objectMeasurementCache> const &measurements)
{
    // initialize output
    Vector3D result;
    // simply call coreFit with at most a handful of measurements, without speed option
    if (measurements.size() > 1)
    {
        float dummy = 0.0; // ignore residuals - that will be dealt with at iterativeTrajectoryFit
        objectCoreFitTriang(measurements, 0, _objectFitCfg, _result, dummy);
        result.x = _result.getX();
        result.y = _result.getY();
        result.z = _result.getZ();
    }
    else
    {
        // save some cycles and copy FCS location from cache
        result = measurements[0].getPositionFcs();
    }
    return result;
}

void objectTracker::iterativeTrajectoryFit(double t)
{
    // iteration was first supposed to be done here, 
    // but due to heavy use of CV functionality,
    // it is now moved to lower library
    
    // fit order fallback: cannot fit a line through a point
    int fitOrder = _objectFitCfg.speedFitOrder;
    if ((int)_positionsFcs.size() < 2)
    {
        fitOrder = 0;
    }

    // other configurables
    int maxIter = _objectFitCfg.outlierMaxIter;
    float nSigma = _objectFitCfg.outlierNSigma;
    float iterFraction = _objectFitCfg.outlierIterFraction;
    
    // calculate
    objectCoreFitTrajectoryIterative(_groupedTime, _positionsFcs, t, fitOrder, maxIter, nSigma, iterFraction, _result, _residual, _numRemoved);
}

void objectTracker::solve(std::vector<objectMeasurementCache> const &measurements, double t, bool withGrouping)
{
    // store pointer to measurements
    _measurementsPtr = &measurements;
    
    if (withGrouping)
    {
        // ball tracking variant
        
        // group measurements in time
        // TODO this needs a caching strategy, which would also need to be robust for delayed incoming measurements
        // without caching we would have quite some expensive re-calculation each iteration
        groupMeasurements();
        
        // for each sub-group, triangulate using corefit, yielding a 3D object position (in FCS)
        _positionsFcs.clear();
        float kpiTriangulation = 0.0;
        for (size_t it = 0; it < _groupedTime.size(); ++it)
        {
            Vector3D posFcs = triangulate(_groupedMeasurements[it]);
            _positionsFcs.push_back(posFcs);
            kpiTriangulation += _groupedMeasurements[it].size();
        }
        if (_groupedTime.size()) // must be, we might as well assert this
        {
            kpiTriangulation /= _groupedTime.size();
            TRACE("kpiTriangulation=%.2f", kpiTriangulation); 
            // if 3 robots are looking at one ball, then ideally this value is 3.0
            // which means each heartbeat each robot provides a measurement, which is used in triangulation
            // future: synchronizing heartbeats on robots (actually: cameras) 
            // should improve this KPI and resulting accuracy
        }
    }
    else
    {
        // obstacle tracking variant
        
        // convert to list of FCS positions so we can fit 
        _positionsFcs.clear();
        _groupedTime.clear();
        for (size_t it = 0; it < measurements.size(); ++it)
        {
            Vector3D posFcs = measurements[it].getPositionFcs();
            _positionsFcs.push_back(posFcs);
            _groupedTime.push_back(measurements[it].getObjectMeasurement().getTimestamp());
        }
    }
    
    // iteratively fit a trajectory (2nd order) through the FCS positions and remove outliers, 
    // which effectively achieves bounce detection and acts as a smoothener
    // * if a bounce just occurred, newest FCS positions will be discarded as outliers
    // * if bounce happened a while ago, FCS position before will be discarded as outliers
    iterativeTrajectoryFit(t);
    
}

