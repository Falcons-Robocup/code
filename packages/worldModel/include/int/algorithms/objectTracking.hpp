// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectTracker.hpp
 *
 *  Created on: Sep 13, 2016
 *      Author: Jan Feitsma
 */

#ifndef OBJECTTRACKER_HPP_
#define OBJECTTRACKER_HPP_


#include <vector>

#include "int/adapters/configurators/WorldModelConfig.hpp"
#include "int/algorithms/objectMeasurementCache.hpp"
#include "int/types/object/objectResultType.hpp"
#include "diagWorldModelLocal.hpp" // from sharedTypes

class objectTracker
{
     
  public:
    objectTracker();
    ~objectTracker();
    
    void solve(std::vector<objectMeasurementCache> const &measurements, double t, bool withGrouping);
    objectResultType getResult() const;
    float getFitResidual() const;
    int getNumRemoved() const;
    int getNumGood() const;
    int getNumBad() const;
    float getTimeSpread() const;
    std::string getDetailsStr() const;
    void setConfig(ConfigWorldModelObjectFit cfg);
    void makeDiagnostics(std::vector<diagObjectMeasurement> &measurements, std::vector<diagObjectMeasurementCluster> &measurementClusters);

  private:
    ConfigWorldModelObjectFit _objectFitCfg;
    objectResultType _result;
    float _residual;
    float _maxSpread;
    float _avgGroupSize;
    int _numRemoved;
    int _numGood;
    int _numBad;
    std::string _detailsStr;
    std::vector<bool> _removedMask;
    std::vector<objectMeasurementCache> const *_measurementsPtr = NULL;
    std::vector<double> _groupedTime;
    std::vector<std::vector<objectMeasurementCache>> _groupedMeasurements;
    std::vector<Vector3D> _positionsFcs;

    void makeDetailsStr();
    void groupMeasurements();
    Vector3D triangulate(std::vector<objectMeasurementCache> const &measurements);
    void iterativeTrajectoryFit(double t);
    
};

#endif /* OBJECTTRACKER_HPP_ */

