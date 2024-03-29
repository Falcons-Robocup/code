// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmObstaclesBlockingBall.hpp
 *
 *  Created on: Jan 23, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMOBSTACLESBLOCKINGBALL_HPP_
#define HMOBSTACLESBLOCKINGBALL_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmObstaclesBlockingBall : public AbstractHeightMap
{
public:
    hmObstaclesBlockingBall();
    virtual ~hmObstaclesBlockingBall();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMOBSTACLESBLOCKINGBALL_HPP_ */
