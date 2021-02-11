// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmObstaclesBlockingOppGoal.hpp
 *
 *  Created on: Jan 6, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMOBSTACLESBLOCKINGOPPGOAL_HPP_
#define HMOBSTACLESBLOCKINGOPPGOAL_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmObstaclesBlockingOppGoal : public abstractHeightMap
{
public:
    hmObstaclesBlockingOppGoal();
    virtual ~hmObstaclesBlockingOppGoal();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMOBSTACLESBLOCKINGOPPGOAL_HPP_ */
