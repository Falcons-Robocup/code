 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cVelocityControlData.cpp
 *
 *  Created on: Jan 14, 2018
 *      Author: Erik Kouters
 */

#include "int/cVelocityControlData.hpp"
#include "cDiagnostics.hpp"
#include "boost/thread/mutex.hpp"

boost::mutex _mtx;

cVelocityControlData::cVelocityControlData()
{
}

void cVelocityControlData::getTargetRobotData(vc_robot_data& targetRobotData)
{
    _mtx.lock();
    targetRobotData = _targetRobotData;
    _mtx.unlock();
}
void cVelocityControlData::setTargetRobotData(const vc_robot_data& targetRobotData)
{
    _mtx.lock();
    _targetRobotData = targetRobotData;
    _mtx.unlock();
}

void cVelocityControlData::getFeedbackRobotData(vc_robot_data& robotData)
{
    _mtx.lock();
    robotData = _feedbackRobotData;
    _mtx.unlock();
}
void cVelocityControlData::setFeedbackRobotData(const vc_robot_data& robotData)
{
    _mtx.lock();
    _feedbackRobotData = robotData;
    _mtx.unlock();
}


// Motors data
void cVelocityControlData::getTargetMotorsData(vc_motors_data& motorsData)
{
    _mtx.lock();
    motorsData = _targetMotorsData;
    _mtx.unlock();
}
void cVelocityControlData::setTargetMotorsData(const vc_motors_data& motorsData)
{
    _mtx.lock();
    _targetMotorsData = motorsData;
    _mtx.unlock();
}
void cVelocityControlData::getFeedbackMotorsData(vc_motors_data& motorsData)
{

    _mtx.lock();
    motorsData = _feedbackMotorsData;
    _mtx.unlock();
}
void cVelocityControlData::setFeedbackMotorsData(const vc_motors_data& motorsData)
{
    _mtx.lock();
    _feedbackMotorsData = motorsData;
    _mtx.unlock();
}




// Motor matrix
void cVelocityControlData::getMatrix(boost::numeric::ublas::matrix<double>& matrix)
{
    _mtx.lock();
    matrix = _matrix;
    _mtx.unlock();
}
void cVelocityControlData::setMatrix(const boost::numeric::ublas::matrix<double>& matrix)
{
    _mtx.lock();
    _matrix = matrix;
    _mtx.unlock();
}

void cVelocityControlData::getInvMatrix(boost::numeric::ublas::matrix<double>& invMatrix)
{
    _mtx.lock();
    invMatrix = _inverseMatrix;
    _mtx.unlock();
}
void cVelocityControlData::setInvMatrix(const boost::numeric::ublas::matrix<double>& invMatrix)
{
    _mtx.lock();
    _inverseMatrix = invMatrix;
    _mtx.unlock();
}
