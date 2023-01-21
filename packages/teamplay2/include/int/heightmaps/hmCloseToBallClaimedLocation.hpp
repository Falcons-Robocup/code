// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmCloseToBallClaimedLocation.hpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMCLOSETOBALLCLAIMEDLOCATION_HPP_
#define HMCLOSETOBALLCLAIMEDLOCATION_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmCloseToBallClaimedLocation : public AbstractHeightMap
{
public:
    hmCloseToBallClaimedLocation();
    virtual ~hmCloseToBallClaimedLocation();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMCLOSETOBALLCLAIMEDLOCATION_HPP_ */
