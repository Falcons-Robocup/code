// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ObjectId.hpp
 *
 *  Created on: Mar 18, 2017
 *      Author: Jan Feitsma
 *
 * Each ball- and obstacle result has a tracker id and is sent from a certain robot.
 * Visualizer uses the id to re-position existing actors.
 */

#ifndef OBJECTID_HPP_
#define OBJECTID_HPP_

#include <stddef.h>
#include <stdint.h>
#include <cassert>

struct ObjectId
{
    uint8_t robotID;
    size_t  trackerID;

    ObjectId() :
    	robotID(0), trackerID(0) {}

    ObjectId(const uint8_t rID, const size_t tID) :
    	robotID(rID), trackerID(tID) {}

    /*
     * Cast to int, for easy operations
     */
    operator int() const
    {
        // hash 2 numbers into one
        //assert(robotID >= 0); // no effect due to using uint8_t
        assert(robotID < 10);
        return (int)(10*trackerID + robotID);
    }
};

#endif /* OBJECTID_HPP_ */

