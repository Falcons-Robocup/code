 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * teamMatesStore.hpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Coen Tempelaars
 */

#ifndef TEAMMATESSTORE_HPP_
#define TEAMMATESSTORE_HPP_

#include "boost/optional.hpp"

#include "int/types/cDecisionTreeTypes.hpp"
#include "int/types/robot.hpp"
#include "int/types/teamMates.hpp"
#include "int/types/intentionType.hpp"


namespace teamplay
{

class teamMatesStore {
public:
    static teamMatesStore& getInstance()
    {
        static teamMatesStore instance;
        return instance;
    }

    // \brief Get all teammates, including the goalie.
    // \note This is a static member. The client can modify the set of robots and/or their properties.
    static teamMates& getTeamMatesIncludingGoalie()
    {
        return getInstance()._team_mates_including_goalie;
    }

    virtual void stashRobot(const int robotNumber);
    virtual void restoreStashedRobot(const treeEnum& newRole);
    void setRobotIntention(intentionStruct intention);
   intentionStruct getRobotIntention(uint8_t robotID) const;

private:
    teamMatesStore();
    virtual ~teamMatesStore();
    teamMatesStore(teamMatesStore const&); // Don't implement
    void operator= (teamMatesStore const&); // Don't implement

    teamMates _team_mates_including_goalie;
    std::vector<intentionStruct> _intentions;
    boost::optional<robot> _stashedRobot;
};


} /* namespace teamplay */

#endif /* TEAMMATESSTORE_HPP_ */
