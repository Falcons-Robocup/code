// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmBetweenPoiAndClosestObstacle.hpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#ifndef HMBETWEENPOIANDCLOSESTOBSTACLE_HPP_
#define HMBETWEENPOIANDCLOSESTOBSTACLE_HPP_

#include "int/heightmaps/abstractHeightMap.hpp"

namespace teamplay
{

class hmBetweenPoiAndClosestObstacle : public abstractHeightMap
{
public:
    hmBetweenPoiAndClosestObstacle();
    virtual ~hmBetweenPoiAndClosestObstacle();

    virtual void precalculate();
    virtual abstractHeightMap refine(const parameterMap_t&);
    virtual std::string getDescription() const;
    virtual std::string getFilename() const;

};

} /* namespace teamplay */

#endif /* HMBETWEENPOIANDCLOSESTOBSTACLE_HPP_ */
