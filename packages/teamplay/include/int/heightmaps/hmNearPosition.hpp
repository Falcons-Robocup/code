// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmNearPosition.hpp
 *
 *  Created on: June 5, 2018
 *      Author: Jan Feitsma
 */

#ifndef HMNEARPOSITION_HPP_
#define HMNEARPOSITION_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmNearPosition : public abstractHeightMap
{
public:
    hmNearPosition();
    virtual ~hmNearPosition();

    virtual void precalculate();
    virtual abstractHeightMap refine(const parameterMap_t&);
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;
};

} /* namespace teamplay */

#endif /* HMNEARPOSITION_HPP_ */

