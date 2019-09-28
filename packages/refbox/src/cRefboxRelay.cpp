 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRefboxRelay.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: Jan Feitsma
 */

#include "int/cRefboxRelay.hpp"
#include "tracing.hpp"
#include "cDiagnostics.hpp"
#include <boost/bind.hpp>

#include "FalconsCommon.h"

static const char TEAM_B = 'B';

cRefboxRelay::cRefboxRelay()
{
    TRACE_FUNCTION("");
    // init RTDB
    _rtdb = RtDB2Store::getInstance().getRtDB2(0);
    _rtdb_sim_team_B = RtDB2Store::getInstance().getRtDB2(0, TEAM_B);
    _refboxConfig.teamColor = refboxConfigTeamColor::CYAN;
    _refboxConfig.field = refboxConfigField::FIELD_LOCALHOST;
    _rtdb->put(REFBOX_CONFIG, &_refboxConfig);
    _matchState.goalsOwn = 0;
    _matchState.goalsOpponent = 0;
    _matchState.phase = matchPhaseEnum::UNKNOWN;
    _matchState.lastRefboxCommand = "";
    _matchState.lastRefboxCommandTime = rtime(0);
    _matchState_sim_team_B = _matchState;

    // simulation specific configuration
    if (getTeamChar() == TEAM_B)
    {
        _doFeedbackLogging = false;
    }

}

cRefboxRelay::~cRefboxRelay()
{
    TRACE_FUNCTION("");
    delete _client;

    // Do not close the RtDB connection
    //delete _rtdb;
}

bool operator!=(refboxConfig const &c1, refboxConfig const &c2) // TODO make generic RTDB template?
{
    std::string s1, s2;
    RtDB2Serializer::serialize(c1, s1);
    RtDB2Serializer::serialize(c2, s2);
    return s1 != s2;
}

bool cRefboxRelay::tick()
{
    TRACE_FUNCTION("");
    _iteration++;
    if (_iteration % _subsampleMSLfeedback == 0)
    {
        _logger.update(_client);
        WRITE_TRACE;
    }

    if (_iteration % _subsampleAttemptReconnect == 0)
    {
        bool needReconnect = (_client == NULL);

        // read config, check for change
        refboxConfig config;
        int life = 0;
        int r = _rtdb->get(REFBOX_CONFIG, &config, life, 0);
        if (r == RTDB2_SUCCESS && (config != _refboxConfig))
        {
            TRACE_INFO("reconfiguring refbox");
            needReconnect = true;
            _refboxConfig = config;
        }

        if (needReconnect)
        {
            if (_client != NULL)
            {
                delete _client;
            }
            createClient();
        }
    }

    // write to RTDB
    if (_iteration % _subsampleRtdbWrite == 0)
    {
        _matchState.currentTime = rtime::now();
        _rtdb->put(MATCH_STATE, &_matchState);

        _matchState_sim_team_B.currentTime = _matchState.currentTime;
        _rtdb_sim_team_B->put(MATCH_STATE, &_matchState_sim_team_B);
    }
    TRACE("Last RefBox command %s", _matchState.lastRefboxCommand);
    WRITE_TRACE;

    return true; // TODO check on some global flag for graceful shutdown?
}

void cRefboxRelay::run()
{
    rtime::loop(_frequency, boost::bind(&cRefboxRelay::tick, this));
}

// callback which gets a string from TCP_IP client
void cRefboxRelay::refboxCallBack(char const *command)
{
    TRACE_FUNCTION(command);
    _matchState.lastRefboxCommand = command;
    _matchState.lastRefboxCommandTime = rtime::now();
    _matchState_sim_team_B.lastRefboxCommand = command;
    _matchState_sim_team_B.lastRefboxCommandTime = _matchState.lastRefboxCommandTime;

    if (_matchState.lastRefboxCommand.find("FIRST_HALF")!= std::string::npos)
    {
        _matchState.phase = matchPhaseEnum::FIRST_HALF;
    }
    if(_matchState.lastRefboxCommand.find("HALF_TIME")!=std::string::npos)
    {
        _matchState.phase = matchPhaseEnum::HALF_TIME;
    }
    if(_matchState.lastRefboxCommand.find("SECOND_HALF")!=std::string::npos)
    {
        _matchState.phase = matchPhaseEnum::SECOND_HALF;
    }
    if(_matchState.lastRefboxCommand.find("END_GAME")!=std::string::npos)
    {
        _matchState.phase = matchPhaseEnum::END_GAME;
    }
    _matchState_sim_team_B.phase = _matchState.phase;

    if(_matchState.lastRefboxCommand.find("_GOAL_OWN")!= std::string::npos)
    {
        _matchState.goalsOwn++;
    }
    if(_matchState.lastRefboxCommand.find("_GOAL_OPP")!= std::string::npos)
    {
        _matchState.goalsOpponent++;
    }

    _matchState_sim_team_B.goalsOwn = _matchState.goalsOwn;
    _matchState_sim_team_B.goalsOpponent = _matchState.goalsOpponent;

    if (_matchState_sim_team_B.lastRefboxCommand.find("OWN") != std::string::npos)
    {
        _matchState_sim_team_B.lastRefboxCommand.replace(_matchState_sim_team_B.lastRefboxCommand.find("OWN"), 3, "OPP");
    }
    else if (_matchState_sim_team_B.lastRefboxCommand.find("OPP") != std::string::npos)
    {
        _matchState_sim_team_B.lastRefboxCommand.replace(_matchState_sim_team_B.lastRefboxCommand.find("OPP"), 3, "OWN");
    }


    // TODO: consider using one shared enum in sharedTypes, but only if it is accompanied with generated enum2str and vice versa
    // something like https://github.com/aantron/better-enums ?
    // TODO: update the following matchState variables, perhaps using some kind of state machine
    // but only if we plan to actually use this (draw in visualizer, or analyzer, or teamplay)
    //    _matchState.goalsOwn++
    //    _matchState.goalsOpponent++
    //    _matchState.phase // is a matchPhaseEnum

    // notify an info event
    TRACE_INFO("refbox: %s", command);
}

void cRefboxRelay::createClient()
{
    TRACE_FUNCTION("");
    std::string server = IPAddress();
    _client = new CTCPIP_Client(server.c_str(), _port, boost::bind(&cRefboxRelay::refboxCallBack, this, _1));
    if (not _client->isConnected())
    {
        delete _client;
        _client = NULL;
        // tick() should try again a few seconds from now
    }
    else
    {
        // set color
        _client->setOwnColor(teamColorString().c_str());
    }
}

std::string cRefboxRelay::teamColorString()
{
    TRACE_FUNCTION("");
    std::string result = "CYAN";
    if (_refboxConfig.teamColor == refboxConfigTeamColor::MAGENTA)
    {
        result = "MAGENTA";
    }
    return result;
}

std::string cRefboxRelay::IPAddress()
{
    TRACE_FUNCTION("");
    std::string result = "localhost";
    switch (_refboxConfig.field)
    {
        case refboxConfigField::FIELD_A:
            result = "172.16.1.2";
            break;
        case refboxConfigField::FIELD_B:
            result = "172.16.2.2";
            break;
        case refboxConfigField::FIELD_C:
            result = "172.16.3.2";
            break;
        case refboxConfigField::FIELD_D:
            result = "172.16.4.2";
            break;
        case refboxConfigField::FIELD_FALCONS:
            result = "172.16.74.10";
            break;
        case refboxConfigField::FIELD_LOCALHOST:
            result = "localhost";
            break;
        default:
            result = "localhost";
            break;
    }
    return result;
}

