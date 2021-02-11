// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectMeasurementCache.hpp
 *
 *  Created on: Oct 30, 2016
 *      Author: Jan Feitsma
 *
 */

#ifndef OBJECTMEASUREMENTCACHE_HPP_
#define OBJECTMEASUREMENTCACHE_HPP_

#include "objectMeasurement.hpp"
#include "falconsCommon.hpp"
#include "opencv2/core/core.hpp"

class objectMeasurementCache
{
    public:
        objectMeasurementCache(objectMeasurement const &objectMeasurement);
        ~objectMeasurementCache();

        objectMeasurement getObjectMeasurement() const;
        Vector3D getPositionFcs() const;
        cv::Mat getCvMatrix() const;

    private:
        // data members
        objectMeasurement _objectMeasurement;
        Vector3D _positionFcs;
        cv::Mat _matrix;
        
        // functions
        void calculateCvMatrix();
};

#endif /* OBJECTMEASUREMENTCACHE_HPP_ */
