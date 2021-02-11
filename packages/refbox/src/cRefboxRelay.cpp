// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRefboxRelay.cpp
 *
 *  Created on: Oct 20, 2010
 *      Author: Jan Feitsma
 */

#include <boost/lexical_cast.hpp>

#include "int/cRefboxRelay.hpp"
#include "tracing.hpp"
#include "cDiagnostics.hpp"
#include <boost/bind.hpp>

#include "falconsCommon.hpp"

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
    T_REFBOX_OVERRIDE noOverride;
    _rtdb->put(REFBOX_OVERRIDE, &noOverride);
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
    int life = 0, r = 0;
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
        r = _rtdb->get(REFBOX_CONFIG, &config, life, 0);
        if (r == RTDB2_SUCCESS)
        {
            // TTA configuration change should not trigger reconnect
            needReconnect |= (config.teamColor != _refboxConfig.teamColor);
            needReconnect |= (config.field != _refboxConfig.field);
            _refboxConfig = config;
        }

        if (needReconnect)
        {
            if (_client != NULL)
            {
                delete _client;
            }
            createClient();
            if (_client != NULL)
            {
                TRACE_INFO("reconfigured refbox");
            }
        }
    }

    // allow overrule from coach laptop
    RtDB2Item item;
    if (_rtdb->getItem(REFBOX_OVERRIDE, item) == RTDB2_SUCCESS)
    {
        T_REFBOX_OVERRIDE rbo = item.value<T_REFBOX_OVERRIDE>();
        if (item.age() < 1.0 && _matchState.lastRefboxCommandTime != item.timestamp && rbo.command.size())
        {
            _matchState.lastRefboxCommand = rbo.command;
            _matchState.lastRefboxCommandTime = item.timestamp;
            TRACE_INFO_TIMEOUT(0.0, "refbox (coach override): %s", rbo.command.c_str());
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
    TRACE("Last RefBox command %s", _matchState.lastRefboxCommand.c_str());
    WRITE_TRACE;

    return true; // TODO check on some global flag for graceful shutdown?
}

void cRefboxRelay::run()
{
    rtime::loop(_frequency, boost::bind(&cRefboxRelay::tick, this));
}

// refbox protocol does not nicely fit with our periodic broadcast
// in case of multi-robot substitution, each robot gets a specific SUBSTITUTION command
// and these are communicated so fast that typically the last one is the one to be sent to our team
// so, we must keep a bit of memory and send all recent substitution target robots
std::string substitutionMemory(std::string const &command)
{
    std::string result = command;
    // do nothing in case command is not SUBSTITUTION
    static std::set<int> substitutionRobots;
    size_t pos = command.find("SUBSTITUTION_OWN");
    if (pos == 0)
    {
        // register target robot
        std::string targetRobotStr = command.substr(pos+17);
        try
        {
            int targetRobot = boost::lexical_cast<int>(targetRobotStr);
            substitutionRobots.insert(targetRobot);
        }
        catch (...)
        {
            TRACE_WARNING("failed to parse target robot from substitition command, str='%s'", targetRobotStr.c_str());
        }
        // add all memorized target robots to command
        result = "SUBSTITUTION_OWN";
        for (auto it = substitutionRobots.begin(); it != substitutionRobots.end(); ++it)
        {
            result += " " + std::to_string(*it);
        }
    }
    else
    {
        // clear memory unless SUBSTITUTION_OPP
        pos = command.find("SUBSTITUTION_OPP");
        if (pos != 0)
        {
            substitutionRobots.clear();
        }
    }
    return result;
}

bool ignoreCommand(std::string const &command)
{
    // hack: ignore SUBSTITUTE_OPP
    // reason: refbox state is written periodically and when we get a burst of singular commands, we typically miss everything except the last ...
    size_t pos = command.find("SUBSTITUTION_OPP");
    if (pos == 0)
    {
        return true;
    }
    return false;
}

// callback which gets a string from TCP_IP client
void cRefboxRelay::refboxCallBack(char const *c)
{
    TRACE_FUNCTION(c);
    if (ignoreCommand(c))
    {
        TRACE("ignoring command: %s", c);
        return;
    }
    std::string command = substitutionMemory(c);
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

    // TODO: instead of using a string for refbox command, better to make a shared enum in sharedTypes? we now have enum2str.

    // notify an info event
    TRACE_INFO_TIMEOUT(0.0, "refbox: %s", command.c_str());
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

