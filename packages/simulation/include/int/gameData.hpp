 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * gameData.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#ifndef GAMEDATA_HPP_
#define GAMEDATA_HPP_

#include <map>
#include <vector>
#include "boost/optional.hpp"

#include "ball.hpp"
#include "obstacle.hpp"
#include "teamData.hpp"
#include "teamID.hpp"

class GameData {
public:
    bool anyRobotHasBall() const;
    bool anyRobotIsMoving() const;

    boost::optional<TeamID> getTeamWithBall() const;
    boost::optional<RobotID> getRobotWithBall(const TeamID) const;

    float getDistanceOfClosestRobotTo(const Point2D&, const TeamID) const;

    /* the following attributes are deliberately public */
    typedef std::map<TeamID, TeamData> Team;
    Team team;
    Ball ball;
    std::vector<Obstacle> nonRobotObstacles; // extra/artificial (handy for testing), visible to both teams
};

#endif /* GAMEDATA_HPP_ */
