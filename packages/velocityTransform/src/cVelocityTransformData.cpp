// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cVelocityTransformData.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Erik Kouters
 */

#include "int/cVelocityTransformData.hpp"
#include "cDiagnostics.hpp"
#include "boost/thread/mutex.hpp"

boost::mutex _mtx;

cVelocityTransformData::cVelocityTransformData()
{
}


void cVelocityTransformData::setConfigAdapter(ConfigRTDBAdapter<ConfigVelocityTransform> *vtConfigAdapter)
{
    _mtx.lock();
    _vtConfigAdapter = vtConfigAdapter;
    _mtx.unlock();
}


// RobotData
void cVelocityTransformData::getTargetRobotData(vt_robot_data& targetRobotData)
{
    _mtx.lock();
    targetRobotData = _targetRobotData;
    _mtx.unlock();
}
void cVelocityTransformData::setTargetRobotData(const vt_robot_data& targetRobotData)
{
    _mtx.lock();
    _targetRobotData = targetRobotData;
    _mtx.unlock();
}

void cVelocityTransformData::getFeedbackRobotData(vt_robot_data& robotData)
{
    _mtx.lock();
    robotData = _feedbackRobotData;
    _mtx.unlock();
}
void cVelocityTransformData::setFeedbackRobotData(const vt_robot_data& robotData)
{
    _mtx.lock();
    _feedbackRobotData = robotData;
    _mtx.unlock();
}


// Motors data
void cVelocityTransformData::getTargetMotorsData(vt_motors_data& motorsData)
{
    _mtx.lock();
    motorsData = _targetMotorsData;
    _mtx.unlock();
}
void cVelocityTransformData::setTargetMotorsData(const vt_motors_data& motorsData)
{
    _mtx.lock();
    _targetMotorsData = motorsData;
    _mtx.unlock();
}
void cVelocityTransformData::getFeedbackMotorsData(vt_motors_data& motorsData)
{

    _mtx.lock();
    motorsData = _feedbackMotorsData;
    _mtx.unlock();
}
void cVelocityTransformData::setFeedbackMotorsData(const vt_motors_data& motorsData)
{
    _mtx.lock();
    _feedbackMotorsData = motorsData;
    _mtx.unlock();
}




// Motor matrix
void cVelocityTransformData::getMatrix(boost::numeric::ublas::matrix<double>& matrix)
{
    _mtx.lock();
    matrix = _matrix;
    _mtx.unlock();
}
void cVelocityTransformData::setMatrix(const boost::numeric::ublas::matrix<double>& matrix)
{
    _mtx.lock();
    _matrix = matrix;
    _mtx.unlock();
}

void cVelocityTransformData::getInvMatrix(boost::numeric::ublas::matrix<double>& invMatrix)
{
    _mtx.lock();
    invMatrix = _inverseMatrix;
    _mtx.unlock();
}
void cVelocityTransformData::setInvMatrix(const boost::numeric::ublas::matrix<double>& invMatrix)
{
    _mtx.lock();
    _inverseMatrix = invMatrix;
    _mtx.unlock();
}
