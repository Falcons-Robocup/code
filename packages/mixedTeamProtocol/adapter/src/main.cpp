// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <string>
#include <stdexcept>

/* Other packages includes */
#include "cDiagnostics.hpp"
#include "tracing.hpp"

#include "falconsCommon.hpp"
#include "FalconsRTDB.hpp"

const std::string RTDB_STORAGE_REFBOX = "/tmp/rtdb_refbox";
const std::string RTDB_DBNAME_REFBOX = "refbox";

std::string receiveRtdbString(RtDB2& rtdb, std::string key)
{
    std::string result;

    auto r = rtdb.get(key, &result);
    if (r != RTDB2_SUCCESS)
    {
        throw(std::runtime_error("Failed to get rtdb key " + key));
    }

    return result;
}

void updateMatchState(matchState& match, std::string command, std::string targetTeam, std::string arguments)
{
    match.lastRefboxCommand = command;
    match.lastRefboxCommandTime = rtime::now();

    if (command.find("FIRST_HALF") != std::string::npos)
    {
        match.phase = matchPhaseEnum::FIRST_HALF;
    }
    if (command.find("HALF_TIME") != std::string::npos)
    {
        match.phase = matchPhaseEnum::HALF_TIME;
    }
    if (command.find("SECOND_HALF") != std::string::npos)
    {
        match.phase = matchPhaseEnum::SECOND_HALF;
    }
    if (command.find("END_GAME") != std::string::npos)
    {
        match.phase = matchPhaseEnum::END_GAME;
    }

    if (command.find("GOAL") != std::string::npos)
    {
        if (targetTeam.find("US") != std::string::npos)
        {
            match.goalsOwn++;
        }
        else
        {
            match.goalsOpponent++;
        }
    }

    if (command.find("SUBGOAL") != std::string::npos)
    {
        if (targetTeam.find("US") != std::string::npos)
        {
            match.goalsOwn--;
        }
        else
        {
            match.goalsOpponent--;
        }
    }

    match.currentTime = rtime::now();
}

int main(int argc, char **argv)
{
    try {
        INIT_TRACE("mtpAdapter");

        matchState match;
        match.goalsOwn = 0;
        match.goalsOpponent = 0;
        match.phase = matchPhaseEnum::UNKNOWN;
        match.lastRefboxCommand = "";
        match.lastRefboxCommandTime = rtime(0);

        auto robotId = getRobotNumber();
        auto teamChar = getTeamChar();

        auto falcons_rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(robotId, teamChar);
        auto refbox_rtdb = RtDB2(RtDB2Context::Builder(robotId)
            .withoutConfigFile()
            .withRootPath(std::string(RTDB_STORAGE_REFBOX) + "_" + std::string(1, teamChar))
            .withDatabase(RTDB_DBNAME_REFBOX)
            .build());

        while (true)
        {
            try {
                falcons_rtdb->put(MATCH_STATE, &match);

                refbox_rtdb.waitForPut("COMMAND");
                std::string command = receiveRtdbString(refbox_rtdb, "COMMAND");
                std::string targetTeam = receiveRtdbString(refbox_rtdb, "TARGETTEAM");
                std::string arguments;

                try {
                    std::string arguments = receiveRtdbString(refbox_rtdb, "ARGUMENTS");
                }
                catch (std::exception &e)
                {
                    // Ignore
                }

                updateMatchState(match, command, targetTeam, arguments);
            }
            catch (std::exception &e)
            {
                TRACE_ERROR("Caught exception: %s", e.what());
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
    }
    catch (...)
    {
        TRACE_ERROR("Caught unknown exception!");
    }

    return 0;
}
