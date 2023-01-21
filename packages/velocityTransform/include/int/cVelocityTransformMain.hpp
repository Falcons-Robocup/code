// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cVelocityTransformMain.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CVELOCITYTRANSFORMMAIN_HPP_
#define CVELOCITYTRANSFORMMAIN_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

#include "falconsCommon.hpp"

#include "int/cVelocityTransformTypes.hpp"
#include "int/cVelocityTransformData.hpp"

// Forward declarations > include in include
class cAbstractVelocityTransform;

/*!
 * \brief The main VelocityTransform class
 */
class cVelocityTransformMain
{
    public:
        /*!
        * \brief The constructor of cVelocityTransformMain
        */
        cVelocityTransformMain();

        /*!
        * \brief The destructor of cVelocityTransformMain
        */
        ~cVelocityTransformMain();

        /*!
        * \brief Performs a single iteration of the VelocityTransform work.
        */
        void iterateFeedback();
        void iterateSetpoint();

        void setAlgorithms();

        cVelocityTransformData* _vtDataClass;


    private:

        /*! \brief The algorithm sequence used */
        cAbstractVelocityTransform *_feedbackAlgorithm;
        cAbstractVelocityTransform *_setpointAlgorithm;

};


#endif /* CVELOCITYTRANSFORMMAIN_HPP_ */
