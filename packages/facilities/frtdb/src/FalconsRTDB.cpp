// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "FalconsRTDB.hpp"


FalconsRTDB::FalconsRTDB(RtDB2Context const &context)
:
    RtDB2(context)
{
}

FalconsRTDB::FalconsRTDB(int agentId, char team)
:
    RtDB2(RtDB2Context::Builder(agentId).withRootPath(std::string(RTDB_STORAGE_TEAMS) + std::string(1, team)).build())
{
}

std::string FalconsRTDB::getPath()
{
    return getContext().getDatabasePath();
}

FalconsRTDB* FalconsRTDBStore::getFalconsRTDB(int db_identifier, char team)
{
    std::string key = std::to_string(db_identifier) + "_path_" + std::string(1, team);
    if (!_rtdbInstances.count(key))
    {
        _rtdbInstances[key] = new FalconsRTDB(RtDB2Context::Builder(db_identifier).withRootPath(std::string(RTDB_STORAGE_TEAMS) + std::string(1, team)).build());
    }
    return _rtdbInstances.at(key);
}

FalconsRTDB* FalconsRTDBStore::getFalconsRTDB(int db_identifier, std::string path)
{
    std::string key = std::to_string(db_identifier) + "_team_" + path;
    if (!_rtdbInstances.count(key))
    {
        _rtdbInstances[key] = new FalconsRTDB(RtDB2Context::Builder(db_identifier).withRootPath(path).build());
    }
    return _rtdbInstances.at(key);
}
