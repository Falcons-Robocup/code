// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectTracker.cpp
 *
 *  Created on: Sep 13, 2016
 *      Author: Jan Feitsma
 */

#include <boost/lexical_cast.hpp>

#include "int/algorithms/objectTracking.hpp"
#include "int/algorithms/objectCoreFit.hpp"

objectTracker::objectTracker()
{
    _residual = 0.0;
    _maxSpread = 0.0;
    _avgGroupSize = 0.0;
    _numGood = 0; 
    _numBad = 0;
    _numRemoved = 0;
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

int objectTracker::getNumGood() const
{
    return _numGood;
}

int objectTracker::getNumBad() const
{
    return _numBad;
}

std::string objectTracker::getDetailsStr() const
{
    return _detailsStr;
}

float objectTracker::getTimeSpread() const
{
    return _maxSpread;
}

void objectTracker::setConfig(ConfigWorldModelObjectFit cfg)
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
    double minT = _measurementsPtr->begin()->getObjectMeasurement().timestamp;
    double sumT = 0.0;
    _maxSpread = 0.0;
    std::vector<objectMeasurementCache> currentGroup;
    // perform grouping, use a grouping tolerance 'groupingDt'
    for (auto it = _measurementsPtr->begin(); it != _measurementsPtr->end(); ++it)
    {
        currT = it->getObjectMeasurement().timestamp;
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

void objectTracker::makeDiagnostics(std::vector<diagObjectMeasurement> &measurements, std::vector<diagObjectMeasurementCluster> &measurementClusters)
{
    // fill in details for diagnostics (plot_balls.py)
    measurements.clear();
    measurementClusters.clear();
    
    return; // JFEI 20200523: disable for now, below assertion very rarely triggers a crash ...
    
    assert(_groupedTime.size() == _positionsFcs.size());
    assert(_groupedTime.size() == _removedMask.size());
    assert(_groupedTime.size() == _groupedMeasurements.size());
    for (size_t it = 0; it < _groupedTime.size(); ++it)
    {
        // cluster
        diagObjectMeasurementCluster c;
        c.t = _groupedTime[it];
        Vector3D posFcs = _positionsFcs[it];
        c.x = posFcs.x;
        c.y = posFcs.y;
        c.z = posFcs.z;
        c.used = (0 != _removedMask[it]);
        measurementClusters.push_back(c);
        // tag raw measurements according to outlier removal
        auto currentGroup = _groupedMeasurements[it];
        for (size_t imeas = 0; imeas < currentGroup.size(); ++imeas)
        {
            diagObjectMeasurement meas;
            meas.m = currentGroup[imeas].getObjectMeasurement();
            meas.used = c.used; // outlier removal is based on cluster
            measurements.push_back(meas);
        }
    }
}

void objectTracker::makeDetailsStr()
{
    // encode data in a string, 1cm/1ms resolution: 
    // numGood numBad (t,x,y,z)*numGood (t,x,y,z)*numBad
    _numGood = 0; 
    _numBad = 0;
    std::string goodList = "", badList = "";
    // TODO remove this assertion

    //tprintf("assert 1: %d ... %d", (int)_groupedTime.size(), (int)_positionsFcs.size());
    //tprintf("assert 2: %d ... %d", (int)_groupedTime.size(), (int)_removedMask.size());
    assert(_groupedTime.size() == _positionsFcs.size());
    assert(_groupedTime.size() == _removedMask.size());
    for (size_t it = 0; it < _groupedTime.size(); ++it)
    {
        double t = _groupedTime[it];
        Vector3D posFcs = _positionsFcs[it];
        char buf[64] = {0};
        sprintf(buf, " %.3f %.2f %.2f %.2f", t, posFcs.x, posFcs.y, posFcs.z);
        if (0 != _removedMask[it])
        {
            badList += buf;
            _numBad++;
        }
        else
        {
            goodList += buf;
            _numGood++;
        }
    }
    _detailsStr = boost::lexical_cast<std::string>(_numGood) + goodList + "   " 
                + boost::lexical_cast<std::string>(_numBad) + badList;
}

void objectTracker::iterativeTrajectoryFit(double t)
{
    // iteration was first supposed to be done here, 
    // but due to heavy use of CV functionality,
    // it is now moved to lower library
    
    // fit order fallback: cannot fit a line through a point
    int fitOrder = _objectFitCfg.speedFitOrder;
    int n        = _positionsFcs.size();
    if (n < 2)
    {
        fitOrder = 0;
    }

    // other configurables
    int maxIter = _objectFitCfg.outlierMaxIter;
    float nSigma = _objectFitCfg.outlierNSigma;
    float iterFraction = _objectFitCfg.outlierIterFraction;
    
    // calculate
    objectCoreFitTrajectoryIterative(_groupedTime, _positionsFcs, t, fitOrder, maxIter, nSigma, iterFraction, _result, _residual, _numRemoved, _removedMask);
    
    // If there is enough measurements and most of the outliers are in the 
    // second half of the measurements, ignore the first half.
    if (n >= 2 * _objectFitCfg.minVmeas)
    {
        int ouliersCountSecondHalf = 0;
        
        for (int i = n / 2; i < n; ++i)
        {
            if (!_removedMask[i])
            {
                ouliersCountSecondHalf++;
            } 
        }
        
        // Ignore the first measurements if there is enough outliers and most of them are in the second half.
        if ((ouliersCountSecondHalf > 2) && (ouliersCountSecondHalf >  2 * _numRemoved / 3))
        {   
            std::vector<double> timeStamps(_groupedTime.begin() + n / 2, _groupedTime.end());
            std::vector<Vector3D> positions(_positionsFcs.begin() + n / 2, _positionsFcs.end());
            objectResultType objectResult(_result);
            float residual;
            int numRemovedTotal;
            std::vector<bool> removedMask(_removedMask.begin() + n / 2, _removedMask.end());
            
            objectCoreFitTrajectoryIterative(timeStamps, positions, t, fitOrder, maxIter, nSigma, iterFraction, objectResult, residual, numRemovedTotal, removedMask);
            
            // TODO: acceptance criteria: check if residual and numRemoved are better?
            if (residual < 0.9 * _residual && numRemovedTotal < (ouliersCountSecondHalf * 9) / 10)
            {
                // log
                //tprintf("First fit (numMeasurements=%d, residual=%.3f, numRemoved=%d, ouliersCountSecondHalf=%d)", n, _residual, _numRemoved, ouliersCountSecondHalf);
                makeDetailsStr();
                //tprintf("All points: %s", _detailsStr.c_str());
                //tprintf("First fit: position: %f %f %f velocity: %f %f %f", _result.getX(), _result.getY(), _result.getZ(), _result.getVX(), _result.getVY(), _result.getVZ());
                
                _residual = residual;
                _numRemoved = numRemovedTotal;
                _removedMask = removedMask;
                _result = objectResult; 
                _positionsFcs.erase(_positionsFcs.begin(), _positionsFcs.begin() + n / 2);
                _groupedTime.erase(_groupedTime.begin(), _groupedTime.begin() + n / 2);
                
                
                //tprintf("Second fit: position: %f %f %f velocity: %f %f %f", _result.getX(), _result.getY(), _result.getZ(), _result.getVX(), _result.getVY(), _result.getVZ());
                // log
                //tprintf("Second fit (residual=%.3f, numRemoved=%d)", residual, numRemovedTotal);
                
            }
        }
    }
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
            //TRACE("kpiTriangulation=%.2f", kpiTriangulation); 
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
            _groupedTime.push_back(measurements[it].getObjectMeasurement().timestamp);
        }
    }
    
    // iteratively fit a trajectory (2nd order) through the FCS positions and remove outliers, 
    // which effectively achieves bounce detection and acts as a smoothener
    // * if a bounce just occurred, newest FCS positions will be discarded as outliers
    // * if bounce happened a while ago, FCS position before will be discarded as outliers
    iterativeTrajectoryFit(t);
    
    // calculate confidence heuristics and tracing details
    makeDetailsStr();
}

