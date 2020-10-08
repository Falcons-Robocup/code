 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * simworld.hpp
 *
 *  Created on: Feb 2, 2019
 *      Author: Coen Tempelaars
 */

#ifndef SIMWORLD_HPP_
#define SIMWORLD_HPP_

#include "abstractConfigAdapter.hpp"
#include "abstractGameDataAdapter.hpp"
#include "abstractMotionAdapter.hpp"
#include "abstractTimeAdapter.hpp"
#include "arbiter.hpp"
#include "simworldGameData.hpp"

class Simworld {
public:
    Simworld();

    void initialize();
    void control();
    void loop();

protected:
    AbstractConfigAdapter* _configAdapter;
    AbstractGameDataAdapter* _gameDataAdapter;
    AbstractMotionAdapter* _motionAdapter;
    AbstractTimeAdapter* _timeAdapter;

private:
    Arbiter _arbiter;
    SimworldGameData _simworldGameData;

    rtime _simulationTime;
    int _sizeTeamA;
    int _sizeTeamB;

    // The number of updates/recalculations of the simulated world per second
    int _tickFrequency;

    // The number of seconds to advance time in a single update (tick)
    double _tick_stepsize_s;

    // The number of milliseconds to sleep inbetween triggering a simulated robot
    // = (1000 / _tickFrequency) / (numRobots + arbiter)
    int _sleeptime_ms;
};

#endif /* SIMWORLD_HPP_ */
