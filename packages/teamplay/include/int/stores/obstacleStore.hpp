// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleStore.hpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#ifndef OBSTACLESTORE_HPP_
#define OBSTACLESTORE_HPP_

#include <vector>

#include "int/types/obstacle.hpp"

namespace teamplay
{

class obstacleStore {
public:
    static obstacleStore& getInstance()
    {
        static obstacleStore instance;
        return instance;
    }

    virtual void clear();
    virtual void addObstacle (const obstacle&);

    virtual std::vector<obstacle> getAllObstacles() const;
    virtual std::vector<obstacle> getAllObstaclesSortedByDistanceTo(const Point2D&) const;

private:
    obstacleStore();
    virtual ~obstacleStore();
    obstacleStore(obstacleStore const&); // Don't implement
    void operator= (obstacleStore const&); // Don't implement

    std::vector<obstacle> _all_obstacles;
};


} /* namespace teamplay */

#endif /* OBSTACLESTORE_HPP_ */
