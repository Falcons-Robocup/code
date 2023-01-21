// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmAvoidTeamMates.hpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#ifndef HMAVOIDTEAMMATES_HPP_
#define HMAVOIDTEAMMATES_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmAvoidTeamMates : public AbstractHeightMap
{
public:
    hmAvoidTeamMates();
    virtual ~hmAvoidTeamMates();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMAVOIDTEAMMATES_HPP_ */
