 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRefboxSignalTypes.hpp
 *
 *  Created on: Sep 8, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CREFBOXSIGNALTYPES_HPP_
#define CREFBOXSIGNALTYPES_HPP_

#include <string>
#include <map>
#include <boost/assign/list_of.hpp>

enum class refboxSignalEnum
{
    INVALID = 0,
    STOP,
    HALT,
    CANCEL,
    READY,
    START,
    RESTART,
    FIRST_HALF,
    HALF_TIME,
    SECOND_HALF,
    END_GAME,
    KICKOFF_OWN,
    KICKOFF_OPP,
    FREE_KICK_OWN,
    FREE_KICK_OPP,
    GOAL_KICK_OWN,
    GOAL_KICK_OPP,
    THROWIN_KICK_OWN,
    THROWIN_KICK_OPP,
    CORNER_KICK_OWN,
    CORNER_KICK_OPP,
    PENALTY_KICK_OWN,
    PENALTY_KICK_OPP,
    GOAL_OWN,
    GOAL_OPP,
    SUBGOAL_OWN,
    SUBGOAL_OPP,
    DUMMY,
    DROPPED_BALL,
    PARK,
    SUBSTITUTION_OWN,

    SIZE_OF_ENUM
};

static std::map<std::string, refboxSignalEnum> refboxSignalMapping = boost::assign::map_list_of
    ("INVALID", refboxSignalEnum::INVALID)
    ("STOP", refboxSignalEnum::STOP)
    ("HALT", refboxSignalEnum::HALT)
    ("CANCEL", refboxSignalEnum::CANCEL)
    ("READY", refboxSignalEnum::READY)
    ("START", refboxSignalEnum::START)
    ("RESTART", refboxSignalEnum::RESTART)
    ("FIRST_HALF", refboxSignalEnum::FIRST_HALF)
    ("HALF_TIME", refboxSignalEnum::HALF_TIME)
    ("SECOND_HALF", refboxSignalEnum::SECOND_HALF)
    ("END_GAME", refboxSignalEnum::END_GAME)
    ("KICKOFF_OWN", refboxSignalEnum::KICKOFF_OWN)
    ("KICKOFF_OPP", refboxSignalEnum::KICKOFF_OPP)
    ("FREEKICK_OWN", refboxSignalEnum::FREE_KICK_OWN)
    ("FREEKICK_OPP", refboxSignalEnum::FREE_KICK_OPP)
    ("GOALKICK_OWN", refboxSignalEnum::GOAL_KICK_OWN)
    ("GOALKICK_OPP", refboxSignalEnum::GOAL_KICK_OPP)
    ("THROWIN_OWN", refboxSignalEnum::THROWIN_KICK_OWN)
    ("THROWIN_OPP", refboxSignalEnum::THROWIN_KICK_OPP)
    ("CORNER_OWN", refboxSignalEnum::CORNER_KICK_OWN)
    ("CORNER_OPP", refboxSignalEnum::CORNER_KICK_OPP)
    ("PENALTY_OWN", refboxSignalEnum::PENALTY_KICK_OWN)
    ("PENALTY_OPP", refboxSignalEnum::PENALTY_KICK_OPP)
    ("GOAL_OWN", refboxSignalEnum::GOAL_OWN)
    ("GOAL_OPP", refboxSignalEnum::GOAL_OPP)
    ("SUBGOAL_OWN", refboxSignalEnum::SUBGOAL_OWN)
    ("SUBGOAL_OPP", refboxSignalEnum::SUBGOAL_OPP)
    ("DUMMY", refboxSignalEnum::DUMMY)
    ("DROPPED_BALL", refboxSignalEnum::DROPPED_BALL)
    ("PARK", refboxSignalEnum::PARK)
    ("SUBSTITUTION_OWN", refboxSignalEnum::SUBSTITUTION_OWN)
    // note SUBSTITUTION_OPP is filtered at refboxRelay to prevent it interfering with SUBSTITUTION_OWN
    // because only the last command would end up here in teamplay...
    // if this is ever needed in teamplay (?), then refboxRelay and the communication to teamplay
    // must be made robust for rapid bursts of different refbox commands at once
    ;

#endif /* CREFBOXSIGNALTYPES_HPP_ */


