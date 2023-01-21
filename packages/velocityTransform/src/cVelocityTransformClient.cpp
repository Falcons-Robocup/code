// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cVelocityTransformClient.cpp
 *
 *  Created on: Nov 13, 2020
 *      Author: Erik Kouters
 */

#include "ext/cVelocityTransformClient.hpp"


cVelocityTransformClient::cVelocityTransformClient()
{
    // Init ConfigAdapter
    _vtConfigAdapter = new ConfigRTDBAdapter<ConfigVelocityTransform>(CONFIG_VELOCITYTRANSFORM);
    std::string vtConfigFile = determineConfig("VelocityTransform");
    _vtConfigAdapter->loadYAML(vtConfigFile);

    // Get config
    ConfigVelocityTransform config;
    _vtConfigAdapter->get(config);

    // Populate matrix from config
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
}

cVelocityTransformClient::~cVelocityTransformClient()
{
    delete _vtConfigAdapter;
}

bool cVelocityTransformClient::exceedsMotorLimits(const robotVelocity& robotVelSetpoint)
{
    bool result = false;

    // Get config
    ConfigVelocityTransform config;
    _vtConfigAdapter->get(config);

    // Do matrix multiplication
	boost::numeric::ublas::vector<double> v(3);
	v(0) = robotVelSetpoint.x;
	v(1) = robotVelSetpoint.y;
	v(2) = robotVelSetpoint.Rz;

	boost::numeric::ublas::vector<double> m = boost::numeric::ublas::prod(_matrix, v);

    // Check if any of the motors exceed the motor velocity limit
    TRACE("motor velocity setpoints: (%8.4f, %8.4f, %8.4f) motorMaxVel: %8.4f", m(0), m(1), m(2), config.motorMaxVel);
    if (fabs(m(0)) > config.motorMaxVel || fabs(m(1)) > config.motorMaxVel || fabs(m(2)) > config.motorMaxVel)
    {
        result = true;
    }

    return result;
}

