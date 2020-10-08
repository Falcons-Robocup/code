 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
#include "ext/cVelocityControlNames.hpp"
#include "int/cVelocityControlData.hpp"
#include "int/cVelocityControlMain.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"

cVelocityControlData _vcData;
cVelocityControlMain _vcMain;

cRTDBInputAdapter _rtdbInputAdapter;
cRTDBOutputAdapter _rtdbOutputAdapter;

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
    vc_robot_data robotData;
    _vcData.getFeedbackRobotData(robotData);

    _rtdbOutputAdapter.setRobotDisplacementFeedback(robotData);
    _rtdbOutputAdapter.setRobotVelocityFeedback(robotData);
}

/*!
 * Used to publish MotorsData target to PeripheralsInterface
 */
void publishTarget()
{
    vc_motors_data target;
    _vcData.getTargetMotorsData(target);

    _rtdbOutputAdapter.setMotorVelocitySetpoint(target);
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
    // Determine YAML to load.
    std::string configFileAbs = determineConfig("VelocityControl");

    INIT_TRACE_HOT_FLUSH;

    try
    {

        // iterate functions
        _iterateFeedbackFunc = boost::bind(&iterateFeedback);
        _iterateSetpointFunc = boost::bind(&iterateSetpoint);

        // Bind publishFeedback function to vcData
        _publishFeedbackFunc = boost::bind(&publishFeedback);
        _vcData.publishFeedback = _publishFeedbackFunc;

        // Bind publishTarget function to vcData
        _publishTargetFunc = boost::bind(&publishTarget);
        _vcData.publishTarget = _publishTargetFunc;

        // RTDB adapters
        _rtdbInputAdapter = cRTDBInputAdapter(&_vcData, _iterateFeedbackFunc, _iterateSetpointFunc);
        _rtdbOutputAdapter = cRTDBOutputAdapter();

        // setup services
        _vcMain._vcDataClass = &_vcData;

        // Load motor matrix
        // TODO - move motor matrix to config file.
        _matrix = boost::numeric::ublas::matrix<double>(3,3);
        _matrix(0,0) = -0.5;
        _matrix(1,0) = -0.5;
        _matrix(2,0) = 1.0;
        _matrix(0,1) = -0.866;
        _matrix(1,1) = 0.866;
        _matrix(2,1) = 0.0;
        _matrix(0,2) = 0.22;
        _matrix(1,2) = 0.22;
        _matrix(2,2) = 0.2266;
        _inverseMatrix = InvertMatrix(_matrix);
        _vcData.setMatrix(_matrix);
        _vcData.setInvMatrix(_inverseMatrix);

        // velocityControl
        _workerThreadFeedback = boost::thread(boost::bind(&cVelocityControlMain::iterateFeedback, &_vcMain));
        _workerThreadSetpoint = boost::thread(boost::bind(&cVelocityControlMain::iterateSetpoint, &_vcMain));

        // wait for motor feedback in a separate thread
        _workerThreadWaitForMotorFeedback = boost::thread(boost::bind(&cRTDBInputAdapter::waitForMotorFeedback, &_rtdbInputAdapter));

        // wait for robot velocity setpoint in the main thread
        _rtdbInputAdapter.waitForRobotVelocitySetpoint();
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
