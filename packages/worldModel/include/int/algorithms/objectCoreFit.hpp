// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectCoreFit.hpp
 *
 *  Created on: Sep 13, 2016
 *      Author: Jan Feitsma
 */

#ifndef OBJECTCOREFIT_HPP_
#define OBJECTCOREFIT_HPP_


#include <vector>
#include "int/adapters/configurators/WorldModelConfig.hpp"
#include "int/algorithms/objectMeasurementCache.hpp"
#include "int/types/object/objectResultType.hpp"

// no class, just functions

void objectCoreFitTriang(
    // inputs
    std::vector<objectMeasurementCache> const &measurements,
    double t, 
    ConfigWorldModelObjectFit config,
    // outputs
    objectResultType &objectResult,
    float &residual
    );

void objectCoreFitTrajectoryIterative(
    // inputs
    std::vector<double> const &timeStamps,
    std::vector<Vector3D> const &positions,
    double t, 
    int fitOrder,
    int maxIter,
    float nSigma,
    float iterFraction,
    // outputs
    objectResultType &objectResult,
    float &residual,
    int &numRemoved,
    std::vector<bool> &removedMask
    );

#endif /* OBJECTCOREFIT_HPP_ */
