 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * SimulationScene.hpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 *
 */

#ifndef SIMULATIONSCENE_HPP_
#define SIMULATIONSCENE_HPP_

#include "pose.hpp"
#include "vec2d.hpp"
#include "vec3d.hpp"


// custom data types w.r.t. the regular ones from worldModel (robotState, ballResult etc), because:
// * pretty serialization (no bare list), readability in rtop and .scene files
// * remove clutter which is to be calculated, focus purely on the things to be manipulated


struct SimulationSceneRobot
{
    int  robotId;
    pose position;
    pose velocity;
    SERIALIZE_DATA(robotId, position, velocity);
};

struct SimulationSceneBall
{
    vec3d position;
    vec3d velocity;
    SERIALIZE_DATA(position, velocity);
};

struct SimulationSceneObstacle
{
    vec2d position;
    vec2d velocity;
    SERIALIZE_DATA(position, velocity);
};

struct SimulationScene
{
    std::vector<SimulationSceneRobot>     robots; // team A
    std::vector<SimulationSceneRobot>     opponents; // team B
    std::vector<SimulationSceneBall>      balls; // 0 or 1
    std::vector<SimulationSceneObstacle>  obstacles; // extra w.r.t. teamB

    SERIALIZE_DATA(robots, opponents, balls, obstacles);
};

#endif

