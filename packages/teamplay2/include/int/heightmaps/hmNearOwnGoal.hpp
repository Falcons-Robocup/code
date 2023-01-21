// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmNearOwnGoal.hpp
 *
 *  Created on: Jun 10, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMNEAROWNGOAL_HPP_
#define HMNEAROWNGOAL_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmNearOwnGoal : public AbstractHeightMap
{
public:
    hmNearOwnGoal();
    virtual ~hmNearOwnGoal();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMNEAROWNGOAL_HPP_ */
