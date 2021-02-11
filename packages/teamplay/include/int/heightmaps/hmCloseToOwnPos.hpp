// Copyright 2017-2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmCloseToOwnPos.hpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#ifndef HMCLOSETOOWNPOS_HPP_
#define HMCLOSETOOWNPOS_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmCloseToOwnPos : public abstractHeightMap
{
public:
    hmCloseToOwnPos();
    virtual ~hmCloseToOwnPos();

    virtual void precalculate();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMCLOSETOOWNPOS_HPP_ */
