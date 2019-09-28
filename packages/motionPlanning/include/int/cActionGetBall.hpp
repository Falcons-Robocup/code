 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionGetBall.hpp
 *
 *  Created on: Feb 3, 2018
 *      Author: Jan Feitsma
 */

#ifndef CACTIONGETBALL_HPP_
#define CACTIONGETBALL_HPP_

#include "int/cAbstractAction.hpp"
#include "int/types/cMotionProfileType.hpp"

class cActionGetBall: public cAbstractAction
{
public:
    actionResultTypeEnum execute();
    void unpackParameters();

private:
    bool _slow                = false;
    bool _ballMovingFastEnough = false;
    bool _needToSprint        = false;
    float _obstacleThreshold  = 0.0;
    float _ballSpeedThreshold = 0.0;
    float _ballSpeedScaling   = 0.0;
    Position2D _target; // intercept target
    Position2D _R;      // robot position
    Velocity2D _Vr;     // robot velocity
    Vector3D   _B;      // ball position
    Vector3D   _Vb;     // ball velocity
    cMotionProfileType _motionProfile = cMotionProfileType::NORMAL;
    
    // helper functions
    void getConfig();
    void initialize();
    void analyzeGeometry();
    void determineMotionProfile();
    bool shouldIntercept();
    void faceBall();
    
};

#endif /* CACTIONGETBALL_HPP_ */

