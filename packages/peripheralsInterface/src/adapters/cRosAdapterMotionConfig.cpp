 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterMotionConfig.cpp
 *
 *  Created on: Feb 11, 2016
 *      Author: Tim Kouters
 */

#include <boost/any.hpp>
#include <int/adapters/cRosAdapterMotionConfig.hpp>
#include <stdexcept>
#include <string>

#include "FalconsCommon.h"

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
		throw runtime_error("Could not inverse motion matrix.");
	}

	// create identity matrix of "inverse"
	inverse.assign(boost::numeric::ublas::identity_matrix<T>(A.size1()));

	// backsubstitute to get the inverse
	lu_substitute(A, pm, inverse);

	return inverse;
 }

/*
 * Class implementation
 */
cRosAdapterMotionConfig::cRosAdapterMotionConfig(PeripheralsInterfaceData &piData) :
		_srv(ros::NodeHandle("~/motors")), _piData(piData) {
}

cRosAdapterMotionConfig::~cRosAdapterMotionConfig() {
}

void cRosAdapterMotionConfig::initialize() {
	/* Bind the reconfiguration function */
	dynamic_reconfigure::Server<peripheralsInterface::motorsConfig>::CallbackType f;
	f = boost::bind(&cRosAdapterMotionConfig::cRosAdapterMotionConfig_cb, this, _1, _2);

	/* Set the callback function as callback for the server. This will automatically trigger a configuration. */
	_srv.setCallback(f);
}

void cRosAdapterMotionConfig::cRosAdapterMotionConfig_cb(
		peripheralsInterface::motorsConfig &config, uint32_t level) {

	// Set the settings to the motorcontrollers.
	MotionBoardSettings boardSettings;
	boardSettings.motorPlotEnabled = config.motorPlotEnabled;
	boardSettings.pid.p = config.Kp;
	boardSettings.pid.i = config.Ki;
	boardSettings.pid.d = config.Kd;
	boardSettings.pid.iTh = config.iTh;
	boardSettings.pid.iMax = config.iMax;
	boardSettings.maxPwmValue = config.PwmMax;
	boardSettings.maxPwmStepValue = config.PwmMaxDeltaSize;
	_piData.getLeftMotionBoard().setSettings(boardSettings);
	_piData.getRightMotionBoard().setSettings(boardSettings);
	_piData.getRearMotionBoard().setSettings(boardSettings);
}
