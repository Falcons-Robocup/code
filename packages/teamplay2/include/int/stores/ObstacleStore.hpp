// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ObstacleStore.hpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#ifndef OBSTACLESTORE_HPP_
#define OBSTACLESTORE_HPP_

#include <vector>

#include "int/types/Obstacle.hpp"

namespace teamplay
{

class ObstacleStore {
public:
    static ObstacleStore& getInstance()
    {
        static ObstacleStore instance;
        return instance;
    }

    virtual void clear();
    virtual void addObstacle (const Obstacle&);

    virtual std::vector<Obstacle> getAllObstacles() const;
    virtual std::vector<Obstacle> getAllObstaclesSortedByDistanceTo(const Point2D&) const;

private:
    ObstacleStore();
    virtual ~ObstacleStore();
    ObstacleStore(ObstacleStore const&); // Don't implement
    void operator= (ObstacleStore const&); // Don't implement

    std::vector<Obstacle> _all_obstacles;
};


} /* namespace teamplay */

#endif /* OBSTACLESTORE_HPP_ */
