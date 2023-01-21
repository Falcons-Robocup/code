// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmNearObstacles.hpp
 *
 *  Created on: Jun 13, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMNEAROBSTACLES_HPP_
#define HMNEAROBSTACLES_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmNearObstacles : public AbstractHeightMap
{
public:
    hmNearObstacles();
    virtual ~hmNearObstacles();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMNEAROBSTACLES_HPP_ */
