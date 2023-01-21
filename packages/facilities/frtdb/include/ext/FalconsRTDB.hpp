// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_FALCONSRTDB_HPP_
#define _INCLUDED_FALCONSRTDB_HPP_

#include <map>

// deal RtDB2 or whichever version we are using via a single Falcons adapter
#include "RtDB2.h"
#include "RtDB2Context.h"


class FalconsRTDB: public RtDB2
{
public:
    FalconsRTDB(RtDB2Context const &context);
    FalconsRTDB(int agentId, char team='A');
    ~FalconsRTDB();

    std::string getPath();
};


class FalconsRTDBStore
{
public:
    static FalconsRTDBStore& getInstance()
    {
        static FalconsRTDBStore instance;
        return instance;
    }

    FalconsRTDBStore(FalconsRTDBStore const&) = delete;
    void operator=(FalconsRTDBStore const&) = delete;

    // legacy/adapter API
    FalconsRTDB* getFalconsRTDB(int db_identifier, char team='A');
    FalconsRTDB* getFalconsRTDB(int db_identifier, std::string databasePath);

    // new rtdb3 API
    //FalconsRTDB* getFalconsRTDB(RtDB2Context const &context); // disabled - constructor of RtDB2Context is too slow at the moment

private:
    FalconsRTDBStore() {}

    std::map<std::string, FalconsRTDB*> _rtdbInstances;

};

// deal all keys and structs
#include "rtdbKeys.hpp"
#include "rtdbStructs.hpp"


// define the storages
#define RTDB_STORAGE_TEAMS ("/tmp/rtdb_team") // prefix, A and B will be appended
#define RTDB_STORAGE_PRODUCTION ("/tmp/rtdb_teamA") // main storage, used on robot; when team is not specified and by rtop/rdump diagnostics tools
#define RTDB_STORAGE_PLAYBACK ("/tmp/rtdb_playback") // used by visualizer/playback, can only log/visualize teamA

#endif
