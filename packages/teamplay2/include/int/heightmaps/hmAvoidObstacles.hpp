// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmAvoidObstacles.hpp
 *
 *  Created on: Apr 28, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMAVOIDOBSTACLES_HPP_
#define HMAVOIDOBSTACLES_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmAvoidObstacles : public AbstractHeightMap
{
public:
    hmAvoidObstacles();
    virtual ~hmAvoidObstacles();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMAVOIDOBSTACLES_HPP_ */
