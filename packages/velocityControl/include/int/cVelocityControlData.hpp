 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cVelocityControlData.hpp
 *
 *  Created on: Jan 14, 2018
 *  Author: Erik Kouters
 */

#ifndef CVELOCITYCONTROLDATA_HPP_
#define CVELOCITYCONTROLDATA_HPP_

#include "linepoint2D.hpp"
#include "polygon2D.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>

#include "cVelocityControlTypes.hpp"

class cVelocityControlData
{
    public:
        cVelocityControlData();
        ~cVelocityControlData() { };

        // Getters and setters

        // Robot data
        // Target from PathPlanning
        // Feedback for WorldModel
        void getTargetRobotData(vc_robot_data& targetRobotData);
        void setTargetRobotData(const vc_robot_data& targetRobotData);

        void getFeedbackRobotData(vc_robot_data& robotData);
        void setFeedbackRobotData(const vc_robot_data& robotData);


        // Motors data
        void getTargetMotorsData(vc_motors_data& motorsData);
        void setTargetMotorsData(const vc_motors_data& motorsData);

        void getFeedbackMotorsData(vc_motors_data& motorsData);
        void setFeedbackMotorsData(const vc_motors_data& motorsData);

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
        vc_robot_data _targetRobotData;
        vc_robot_data _feedbackRobotData;

        // Motors data
        vc_motors_data _targetMotorsData;
        vc_motors_data _feedbackMotorsData;

        // Motor matrix
        boost::numeric::ublas::matrix<double> _matrix;
        boost::numeric::ublas::matrix<double> _inverseMatrix;

};

#endif /* CVELOCITYCONTROLDATA_HPP_ */
