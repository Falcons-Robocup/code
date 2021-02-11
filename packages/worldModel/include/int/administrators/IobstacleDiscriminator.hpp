// Copyright 2018-2019 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IobstacleDiscriminator.hpp
 *
 *  Created on: Oct 24, 2018
 *      Author: lucas
 */

#ifndef IOBSTACLEDISCRIMINATOR_HPP_
#define IOBSTACLEDISCRIMINATOR_HPP_

#include <vector>

#include "int/types/robot/robotType.hpp"
#include "obstacleMeasurement.hpp"
#include "int/types/obstacle/obstacleType.hpp"
#include "diagWorldModel.hpp"


class IobstacleDiscriminator
{
public:
    virtual ~IobstacleDiscriminator(){}

    virtual void addMeasurement(const obstacleMeasurement& measurement) = 0;
    virtual void performCalculation(rtime const timeNow, const std::vector<robotClass_t>& teamMembers) = 0;
    virtual std::vector<obstacleClass_t> getObstacles() const = 0;

    virtual int numTrackers() const = 0;

    virtual void fillDiagnostics(diagWorldModel &diagnostics) = 0;
};

#endif /* IOBSTACLEDISCRIMINATOR_HPP_ */
