// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPathPlanningInterface.hpp
 *
 *  Created on: Nov 21, 2017
 *      Author: Jan Feitsma
 */

#ifndef CPATHPLANNINGINTERFACE_HPP_
#define CPATHPLANNINGINTERFACE_HPP_

#include <map>
#include "motionTypeEnum.hpp"
#include "int/types/cForbiddenAreaType.hpp"
#include "falconsCommon.hpp"

class cPathPlanningInterface
{
  public:
    cPathPlanningInterface() {};
    virtual ~cPathPlanningInterface() {};

    virtual void moveTo(Position2D const &pos, motionTypeEnum motionType = motionTypeEnum::NORMAL) = 0;
    virtual void stop() = 0;

    void addForbiddenArea(cForbiddenAreaType const &area, double expireTimestamp);
    void clearForbiddenAreas();
    std::vector<cForbiddenAreaType> getForbiddenAreas();
    
  private:
    std::map<cForbiddenAreaType, double> _forbiddenAreas; // 2nd: expire timestamp
    void cleanup();
    
};

#endif /* CPATHPLANNINGINTERFACE_HPP_ */

