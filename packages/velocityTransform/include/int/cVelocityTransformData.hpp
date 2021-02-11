// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cVelocityTransformData.hpp
 *
 *  Created on: Jan 14, 2018
 *  Author: Erik Kouters
 */

#ifndef CVELOCITYTRANSFORMDATA_HPP_
#define CVELOCITYTRANSFORMDATA_HPP_

#include "linepoint2D.hpp"
#include "polygon2D.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include "cVelocityTransformTypes.hpp"

class cVelocityTransformData
{
    public:
        cVelocityTransformData();
        ~cVelocityTransformData() { };

        // Getters and setters

        // Robot data
        // Target from VelocityControl
        // Feedback for WorldModel
        void getTargetRobotData(vt_robot_data& targetRobotData);
        void setTargetRobotData(const vt_robot_data& targetRobotData);

        void getFeedbackRobotData(vt_robot_data& robotData);
        void setFeedbackRobotData(const vt_robot_data& robotData);


        // Motors data
        void getTargetMotorsData(vt_motors_data& motorsData);
        void setTargetMotorsData(const vt_motors_data& motorsData);

        void getFeedbackMotorsData(vt_motors_data& motorsData);
        void setFeedbackMotorsData(const vt_motors_data& motorsData);

        // Motor matrix
        void getMatrix(boost::numeric::ublas::matrix<double>& matrix);
        void setMatrix(const boost::numeric::ublas::matrix<double>& matrix);

        void getInvMatrix(boost::numeric::ublas::matrix<double>& invMatrix);
        void setInvMatrix(const boost::numeric::ublas::matrix<double>& invMatrix);

        publishFeedbackFunctionType publishFeedback;

        publishTargetFunctionType publishTarget;

    private:

        // Data types
        // target is what should be published to peripheralsInterface
        // feedback is what is received from peripheralsInterface

        // Robot data (RCS)
        vt_robot_data _targetRobotData;
        vt_robot_data _feedbackRobotData;

        // Motors data
        vt_motors_data _targetMotorsData;
        vt_motors_data _feedbackMotorsData;

        // Motor matrix
        boost::numeric::ublas::matrix<double> _matrix;
        boost::numeric::ublas::matrix<double> _inverseMatrix;

};

#endif /* CVELOCITYTRANSFORMDATA_HPP_ */
