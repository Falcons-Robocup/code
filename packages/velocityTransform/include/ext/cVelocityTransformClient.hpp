// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cVelocityTransformClient.hpp
 *
 *  Created on: Nov 13, 2020
 *      Author: Erik Kouters
 */

#ifndef CVELOCITYTRANSFORMCLIENT_HPP_
#define CVELOCITYTRANSFORMCLIENT_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <sstream>

#include "falconsCommon.hpp"
#include "FalconsRTDB.hpp"
#include "ConfigRTDBAdapter.hpp"


class cVelocityTransformClient
{
    public:
        /*!
        * \brief The constructor of cVelocityTransformClient
        */
        cVelocityTransformClient();

        /*!
        * \brief The destructor of cVelocityTransformClient
        */
        ~cVelocityTransformClient();

        /*!
        * \brief Checks if the given robot velocity setpoint will exceed any motor limits
        */
        bool exceedsMotorLimits(const robotVelocity& robotVelSetpoint);

    private:

    ConfigRTDBAdapter<ConfigVelocityTransform>* _vtConfigAdapter;
    
    boost::numeric::ublas::matrix<double> _matrix;

};


#endif /* CVELOCITYTRANSFORMCLIENT_HPP_ */
