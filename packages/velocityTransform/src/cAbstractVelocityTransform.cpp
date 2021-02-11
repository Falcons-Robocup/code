// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractVelocityTransform.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#include "int/cAbstractVelocityTransform.hpp"
#include <boost/thread/thread.hpp>

cAbstractVelocityTransform::cAbstractVelocityTransform(cVelocityTransformMain* main)
{
    TRACE(">");
    _vtMain = main;
    _dt = 0.0; // Recalculated every iteration due to event-based velocityTransform.
    _prevTimestamp = rtime::now();
    _prev_vel = Velocity2D();
    TRACE("<");
}

cAbstractVelocityTransform::~cAbstractVelocityTransform()
{
}

void cAbstractVelocityTransform::execute()
{
    TRACE_FUNCTION("");


    std::list<cAbstractVelocityTransform*>::iterator it;
    for (it = _vtBlocks.begin(); it != _vtBlocks.end(); ++it)
    {
        // Compute new _dt for each algorithm
        //(*it)->computeDt();

        //(*it)->setData(_vtData);
        (*it)->execute();
        //_vtData = (*it)->getData();
    }

    //_main->_vtDataClass->publishSpeed(_vtData.vel);
}

void cAbstractVelocityTransform::setData(vt_data_struct_t &vtData)
{
    _vtData = vtData;
}

vt_data_struct_t cAbstractVelocityTransform::getData()
{
    return _vtData;
}

void cAbstractVelocityTransform::computeDt()
{
    // Compute new _dt

    rtime time_now = rtime::now(); // TODO this is not simulator- and test-friendly, better to move timestamping outside

    _dt = double(time_now - _prevTimestamp);

    // If pathplanning is not used (e.g., robot is stopped), _dt is not recomputed
    // The next time pathplanning is triggered, _dt will be very large (seconds).
    // If this happens, set _dt to 1/30.
    if (_dt > 0.2)
    {
        _dt = 1.0/30.0;
    }

    _prevTimestamp = time_now;
}
