// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractVelocityTransform.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CABSTRACTVELOCITYTRANSFORM_HPP_
#define CABSTRACTVELOCITYTRANSFORM_HPP_

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

#include "tracing.hpp"
#include "FalconsRtDB2.hpp" // for rtime

#include "int/cVelocityTransformMain.hpp"

/*!
 * \brief is the abstract class that each VelocityTransform algorithm should inherit.
 *
 * cAbstractVelocityTransform offers the virtual functions updateVelocityTransformXY() and updateVelocityTransformPhi()
 * that decouple XY movement from turning movement. This allows the VelocityTransform algorithm
 * to specify a different algorithm for XY movement than for turning movement.
 */
class cAbstractVelocityTransform
{
    public:
        /*!
         * \brief The constructor of cAbstractVelocityTransform
         *
         * \param[in] main The parent class
         */
        cAbstractVelocityTransform(cVelocityTransformMain* main);

        /*!
         * \brief The destructor of cAbstractVelocityTransform
         */
        virtual ~cAbstractVelocityTransform();

        virtual void execute();

        void setData(vt_data_struct_t &vtData);
        vt_data_struct_t getData();

        void computeDt();

        std::list<cAbstractVelocityTransform*> _vtBlocks;

        /*! \brief The delta time for every interval */
        double             _dt;

    protected:

        /*! \brief The parent class */
        cVelocityTransformMain* _vtMain;

        rtime _prevTimestamp;

        /*! \brief The velocity of the previous interval. Often used by the different algorithms. */
        Velocity2D         _prev_vel;

        vt_data_struct_t _vtData;

};


#endif /* CABSTRACTVELOCITYTRANSFORM_HPP_ */
