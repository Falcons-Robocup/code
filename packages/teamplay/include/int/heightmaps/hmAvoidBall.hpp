// Copyright 2018 Ivo Matthijssen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmAvoidBall.hpp
 *
 *  Created on: Jun 05, 2018
 *      Author: Ivo Matthijssen
 */

#ifndef HMAVOIDBALL_HPP_
#define HMAVOIDBALL_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmAvoidBall : public abstractHeightMap
{
public:
    hmAvoidBall();
    virtual ~hmAvoidBall();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMAVOIDBALL_HPP_ */
