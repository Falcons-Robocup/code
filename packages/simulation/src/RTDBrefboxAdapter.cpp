 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBrefboxAdapter.cpp
 *
 *  Created on: March 4, 2019
 *      Author: Coen Tempelaars
 */

#include "int/RTDBrefboxAdapter.hpp"
#include "int/RTDBaccess.hpp"
#include "int/generated_enum2str.hpp"

#include <stdexcept>
#include "boost/format.hpp"

#include "FalconsRtDB2.hpp"
#include "tracing.hpp"


RTDBRefBoxAdapter::RTDBRefBoxAdapter()
: _command_for_team_A("")
, _command_for_team_B("")
, _score_for_team_A(0)
, _score_for_team_B(0)
{ }

void RTDBRefBoxAdapter::sendCommand () const
{
    T_MATCH_STATE matchState;
    matchState.currentTime = rtime::now();
    matchState.goalsOwn = _score_for_team_A;
    matchState.goalsOpponent = _score_for_team_B;

    auto rtdbConnection = getRTDBConnection(TeamID::A);
    matchState.lastRefboxCommand = _command_for_team_A;
    tprintf("put MATCH_STATE (Team A) lastRefboxCommand=%s", matchState.lastRefboxCommand.c_str());
    auto r = rtdbConnection->put(MATCH_STATE, &matchState);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing MATCH_STATE (refbox signal) to RtDB for team A");
    }

    rtdbConnection = getRTDBConnection(TeamID::B);
    matchState.lastRefboxCommand = _command_for_team_B;
    tprintf("put MATCH_STATE (Team B) lastRefboxCommand=%s", matchState.lastRefboxCommand.c_str());
    r = rtdbConnection->put(MATCH_STATE, &matchState);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing MATCH_STATE (refbox signal) to RtDB for team B");
    }
}

void RTDBRefBoxAdapter::registerGoal (const TeamID& teamID)
{
    if (teamID == TeamID::A)
    {
        _score_for_team_A += 1;
    }
    else
    {
        _score_for_team_B += 1;
    }
}

void RTDBRefBoxAdapter::republish() const
{
    TRACE_FUNCTION("");
    sendCommand();
}

void RTDBRefBoxAdapter::sendStart()
{
    TRACE_FUNCTION("");
    _command_for_team_A = "COMM_START";
    _command_for_team_B = "COMM_START";
    sendCommand();
}


void RTDBRefBoxAdapter::sendStop()
{
    TRACE_FUNCTION("");
    _command_for_team_A = "COMM_STOP";
    _command_for_team_B = "COMM_STOP";
    sendCommand();
}


void RTDBRefBoxAdapter::sendSetpiece (const Setpiece& setpiece, const TeamID& teamID)
{
    TRACE_FUNCTION("");

    if (setpiece == Setpiece::DROPPED_BALL)
    {
        _command_for_team_A = "COMM_DROPPED_BALL";
        _command_for_team_B = "COMM_DROPPED_BALL";
    }
    else
    {
        auto infix = enum2str(setpiece);

        auto suffix_for_team_A = std::string();
        auto suffix_for_team_B = std::string();

        if (teamID == TeamID::A)
        {
            suffix_for_team_A = "OWN";
            suffix_for_team_B = "OPP";
        }
        else
        {
            suffix_for_team_A = "OPP";
            suffix_for_team_B = "OWN";
        }

        _command_for_team_A = (boost::format("COMM_%1%_%2%") % infix % suffix_for_team_A).str();
        _command_for_team_B = (boost::format("COMM_%1%_%2%") % infix % suffix_for_team_B).str();
    }

    sendCommand();
}
