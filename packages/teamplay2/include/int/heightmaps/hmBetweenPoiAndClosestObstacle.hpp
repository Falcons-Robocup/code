// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmBetweenPoiAndClosestObstacle.hpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMBETWEENPOIANDCLOSESTOBSTACLE_HPP_
#define HMBETWEENPOIANDCLOSESTOBSTACLE_HPP_

#include "int/heightmaps/AbstractHeightMap.hpp"

namespace teamplay
{

class hmBetweenPoiAndClosestObstacle : public AbstractHeightMap
{
public:
    hmBetweenPoiAndClosestObstacle();
    virtual ~hmBetweenPoiAndClosestObstacle();

    virtual void precalculate();
    virtual AbstractHeightMap refine();
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMBETWEENPOIANDCLOSESTOBSTACLE_HPP_ */
