// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmObstaclesBlockingTeammates.hpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMOBSTACLESBLOCKINGTEAMMATES_HPP_
#define HMOBSTACLESBLOCKINGTEAMMATES_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmObstaclesBlockingTeammates : public AbstractHeightMap
{
public:
    hmObstaclesBlockingTeammates();
    virtual ~hmObstaclesBlockingTeammates();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMOBSTACLESBLOCKINGTEAMMATES_HPP_ */
