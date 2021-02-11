// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmObstaclesBlockingTeammates.hpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMOBSTACLESBLOCKINGTEAMMATES_HPP_
#define HMOBSTACLESBLOCKINGTEAMMATES_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmObstaclesBlockingTeammates : public abstractHeightMap
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
