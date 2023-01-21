// Copyright 2019-2022 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IballDiscriminator.hpp
 *
 *  Created on: Nov 26, 2019
 *      Author: lucas
 */

#ifndef IBALLDISCRIMINATOR_HPP_
#define IBALLDISCRIMINATOR_HPP_

#include <vector>

#include "diagWorldModel.hpp"
#include "ballMeasurement.hpp"
#include "int/types/ball/ballType.hpp"
#include "vector3d.hpp"

class IballDiscriminator
{
public:
    virtual ~IballDiscriminator(){}

    virtual void addMeasurement(const ballMeasurement& measurement) = 0;
    virtual void addPossessionMeasurement(const Vector3D& ball_pos, uint8_t robotID, rtime timestamp) = 0;
    virtual void performCalculation(rtime timeNow, const Vector2D& pos) = 0;

    virtual std::vector<ballClass_t> getBalls() const = 0;
    virtual void getMeasurementsToSync(std::vector<ballMeasurement>& measurements) = 0;
    virtual void fillDiagnostics(diagWorldModel& diagnostics) = 0;
};

#endif /* IOBSTACLEDISCRIMINATOR_HPP_ */
