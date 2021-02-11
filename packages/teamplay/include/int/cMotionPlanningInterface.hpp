// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cMotionPlanningInterface.hpp
 *
 *  Created on: Nov 25, 2017
 *      Author: Jan Feitsma
 */

#ifndef CMOTIONPLANNINGINTERFACE_HPP_
#define CMOTIONPLANNINGINTERFACE_HPP_

#include "polygon2D.hpp"
#include "falconsCommon.hpp"

enum class mpStatusEnum
{
    UNDEFINED = 0,
    RUNNING,
    PASSED,
    FAILED,
    ERROR
};

class cMotionPlanningInterface
{
public:
    cMotionPlanningInterface() {};
    virtual ~cMotionPlanningInterface() {};

    virtual void connect () = 0;

    virtual mpStatusEnum moveTo(Position2D targetPos, const std::vector<polygon2D>& forbiddenAreas = {}, bool slow = false) = 0;
    virtual mpStatusEnum passTo(float x, float y) = 0;
    virtual mpStatusEnum shootAt(float x, float y, float z) = 0;
    virtual mpStatusEnum lobShot(float x, float y, float z) = 0;
    virtual void stop() = 0;
    virtual void suppressBallHandlers() = 0;
    virtual mpStatusEnum getBall(bool slow) = 0;

};

#endif /* CMOTIONPLANNINGINTERFACE_HPP_ */
