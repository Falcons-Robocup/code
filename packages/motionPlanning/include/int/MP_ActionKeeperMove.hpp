 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionKeeperMove.hpp
 *
 *  Created on: Apr 26, 2018
 *      Author: Erik Kouters
 */

#ifndef MP_ACTIONKEEPERMOVE_HPP_
#define MP_ACTIONKEEPERMOVE_HPP_

#include "MP_AbstractAction.hpp"


class MP_ActionKeeperMove: public MP_AbstractAction
{

public:
    MP_ActionKeeperMove();
    actionResultTypeEnum execute();
    
private:
    void unpackParameters();
    void getCfg();
    void calculateSlow();
    void checkBallBehindKeeper();
    void clipTarget();
    void checkExtendKeeperFrame();
    void checkDisableObstacleAvoidance();
    
    // data members
    actionResultTypeEnum _result;
    Position2D _currentPos;
    Position2D _targetPos;
    bool       _slow = false;
    bool       _haveKeeperFrame = false;
    bool       _disabledObstacleAvoidance = false;
    
    float _Y_MAX_OFFSET_KEEPER = 0.0;
    float _GOALPOST_OFFSET_KEEPER = 0.0;
    float _FRAME_EXTENSION_SPEED_THRESHOLD = 3.0;
    float _FRAME_SIDE_EXTENSION_TOLERANCE = 0.2;
    float _FRAME_TOP_EXTENSION_TOLERANCE = 0.0;

};

#endif /* MP_ACTIONKEEPERMOVE_HPP_ */
