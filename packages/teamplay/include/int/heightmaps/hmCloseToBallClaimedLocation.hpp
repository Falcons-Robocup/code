// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmCloseToBallClaimedLocation.hpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMCLOSETOBALLCLAIMEDLOCATION_HPP_
#define HMCLOSETOBALLCLAIMEDLOCATION_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmCloseToBallClaimedLocation : public abstractHeightMap
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
