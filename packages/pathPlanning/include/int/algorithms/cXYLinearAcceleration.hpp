 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPIDPathPlanning.hpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#ifndef CXYLINEARACCELERATION_HPP_
#define CXYLINEARACCELERATION_HPP_

#include "int/cAbstractPathPlanning.hpp"
//#include "pathPlanning/s_pathplanning_setpointGeneratorType.h"

class cXYLinearAcceleration : public cAbstractPathPlanning
{
    public:
        cXYLinearAcceleration(cPathPlanningMain* main) : cAbstractPathPlanning(main)
        {
            _n.reset(new ros::NodeHandle());
            _isBraking = false;
            //_s_getJerk = _n->serviceClient<pathPlanning::s_pathplanning_setpointGeneratorType>("getJerk", USE_SERVICE_PERSISTENCY);
        };
        ~cXYLinearAcceleration() { };

        void execute();

    private:

        double getBrakeDistance(Velocity2D currVel);
        std::vector< std::vector<double> > calcNewTrajectory(double startpos, double endpos, double startvel, double endvel, double maxV, double maxA, double maxJ);
        double getNewVel(std::vector< std::vector<double> > polynomials, rtime timeComputed);

        Position2D _currTarget;
        bool        _isBraking;

        std::vector< std::vector<double> > trajectoryX;
        std::vector< std::vector<double> > trajectoryY;

        timeval trajectoryXComputed;
        timeval trajectoryYComputed;

        double prevAccX;
        double prevExpAccX; // expected Acceleration from SPG
        double prevVelX;
        double prevPosX;

        double prevAccY;
        double prevExpAccY; // expected Acceleration from SPG
        double prevVelY;
        double prevPosY;

        //ros::ServiceClient _s_getJerk;
        boost::shared_ptr<ros::NodeHandle> _n;
        pp_setpoint_output newJerk2(double startpos, double startvel, double startacc, double endpos, double endvel, double endacc, double maxV, double maxA, double maxJ);
        pp_setpoint_output newAcc_2ndorder(double startpos, double startvel, double endpos, double endvel, double maxV, double maxA);
};

#endif /* CXYLINEARACCELERATION_HPP_ */
