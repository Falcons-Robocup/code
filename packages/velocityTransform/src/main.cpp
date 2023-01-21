// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Erik Kouters
 */

#include <boost/thread.hpp>

// Falcons facilities
#include "cDiagnostics.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"

// internal
#include "int/cVelocityTransformData.hpp"
#include "int/cVelocityTransformMain.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"

cVelocityTransformData _vtData;
cVelocityTransformMain _vtMain;

cRTDBInputAdapter *_rtdbInputAdapter = NULL;
cRTDBOutputAdapter *_rtdbOutputAdapter = NULL;

boost::thread _workerThreadFeedback;
boost::thread _workerThreadSetpoint;

boost::numeric::ublas::matrix<double> _matrix;
boost::numeric::ublas::matrix<double> _inverseMatrix;

iterateFunctionType _iterateFeedbackFunc;
publishFeedbackFunctionType _publishFeedbackFunc;

iterateFunctionType _iterateSetpointFunc;
publishTargetFunctionType _publishTargetFunc;

boost::thread _workerThreadWaitForMotorFeedback;


void iterateFeedback()
{
    _workerThreadFeedback.interrupt();
}

void iterateSetpoint()
{
    _workerThreadSetpoint.interrupt();
}

/*!
 * Used to publish RobotData feedback to WorldModel
 */
void publishFeedback()
{
    vt_robot_data robotData;
    _vtData.getFeedbackRobotData(robotData);

    if (_rtdbOutputAdapter != NULL)
    {
        _rtdbOutputAdapter->setRobotDisplacementFeedback(robotData);
        _rtdbOutputAdapter->setRobotVelocityFeedback(robotData);
    }
}

/*!
 * Used to publish MotorsData target to PeripheralsInterface
 */
void publishTarget()
{
    vt_motors_data target;
    _vtData.getTargetMotorsData(target);

    if (_rtdbOutputAdapter != NULL)
    {
        _rtdbOutputAdapter->setMotorVelocitySetpoint(target);
    }
}

template<class T> static boost::numeric::ublas::matrix<T> InvertMatrix (const boost::numeric::ublas::matrix<T>& input) {
    boost::numeric::ublas::matrix<T> inverse(input.size1(), input.size2());

    // create a working copy of the input
    boost::numeric::ublas::matrix<T> A(input);
    // create a permutation matrix for the LU-factorization
    boost::numeric::ublas::permutation_matrix<std::size_t> pm(A.size1());

    // perform LU-factorization
    int res = lu_factorize(A,pm);
    if( res != 0 )
    {
        throw std::runtime_error("Could not inverse motion matrix.");
    }

    // create identity matrix of "inverse"
    inverse.assign(boost::numeric::ublas::identity_matrix<T>(A.size1()));

    // backsubstitute to get the inverse
    lu_substitute(A, pm, inverse);

    return inverse;
}

int main(int argc, char **argv)
{

    INIT_TRACE("velocityTransform");

    try
    {

        // iterate functions
        _iterateFeedbackFunc = boost::bind(&iterateFeedback);
        _iterateSetpointFunc = boost::bind(&iterateSetpoint);

        // Bind publishFeedback function to vcData
        _publishFeedbackFunc = boost::bind(&publishFeedback);
        _vtData.publishFeedback = _publishFeedbackFunc;

        // Bind publishTarget function to vcData
        _publishTargetFunc = boost::bind(&publishTarget);
        _vtData.publishTarget = _publishTargetFunc;

        // RTDB adapters
        _rtdbInputAdapter = new cRTDBInputAdapter(&_vtData, _iterateFeedbackFunc, _iterateSetpointFunc);
        _rtdbOutputAdapter = new cRTDBOutputAdapter();

        // setup services
        _vtMain._vtDataClass = &_vtData;

        // setup config
        ConfigRTDBAdapter<ConfigVelocityTransform> *vtConfigAdapter = new ConfigRTDBAdapter<ConfigVelocityTransform>(CONFIG_VELOCITYTRANSFORM);
        std::string vtConfigFile = determineConfig("VelocityTransform");
        vtConfigAdapter->loadYAML(vtConfigFile);
        _vtData.setConfigAdapter(vtConfigAdapter);

        // get config
        ConfigVelocityTransform config;
        vtConfigAdapter->get(config);

        // Load motor matrix from config
        _matrix = boost::numeric::ublas::matrix<double>(3,3);
        _matrix(0,0) = config.matrix[0][0];
        _matrix(1,0) = config.matrix[1][0];
        _matrix(2,0) = config.matrix[2][0];
        _matrix(0,1) = config.matrix[0][1];
        _matrix(1,1) = config.matrix[1][1];
        _matrix(2,1) = config.matrix[2][1];
        _matrix(0,2) = config.matrix[0][2];
        _matrix(1,2) = config.matrix[1][2];
        _matrix(2,2) = config.matrix[2][2];
        _inverseMatrix = InvertMatrix(_matrix);
        _vtData.setMatrix(_matrix);
        _vtData.setInvMatrix(_inverseMatrix);

        // velocityTransform
        _workerThreadFeedback = boost::thread(boost::bind(&cVelocityTransformMain::iterateFeedback, &_vtMain));
        _workerThreadSetpoint = boost::thread(boost::bind(&cVelocityTransformMain::iterateSetpoint, &_vtMain));

        // wait for motor feedback in a separate thread
        _workerThreadWaitForMotorFeedback = boost::thread(boost::bind(&cRTDBInputAdapter::waitForMotorFeedback, _rtdbInputAdapter));

        // wait for robot velocity setpoint in the main thread
        _rtdbInputAdapter->waitForRobotVelocitySetpoint();
    }
    catch (std::exception &e)
    {
        std::cerr << "Error occurred:" << e.what() << std::endl;
        TRACE_ERROR("Error occurred: %s", e.what());
        return 1;
    }
    std::cerr << "Exiting..." << std::endl;
    return 0;
}
