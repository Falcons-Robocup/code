// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "int/AdapterRTDB.hpp"

// headers from other packages
#include "RtDB2Monitor.h"

// standard/system headers
// ...

using namespace mtp;

AdapterRTDB::AdapterRTDB(PlayerId const &id, std::string const &dbname, bool path_encoding)
:
    RtDB2(createContext(id, dbname, path_encoding))
{
    auto const &c = getContext();
    std::ostringstream os;
    os << c;
    //tprintf("RTDB configuration:\n%s", os.str().c_str()); // TODO: put this behind a MTP_VERBOSE compilation flag?
}

AdapterRTDB::~AdapterRTDB()
{
}

std::set<int> AdapterRTDB::getClients() // TODO: const
{
    std::set<int> result = getAgentIds(); // using new RtDB2Monitor
    return result;
}

RtDB2Context AdapterRTDB::createContext(PlayerId const &id, std::string const &dbname, bool path_encoding)
{
    if (path_encoding)
    {
        return RtDB2Context::Builder(id.shirtId)
            .withoutConfigFile()
            .withRootPath(MTP_RTDB_STORAGE_PATH +
                std::string(dbname) +
                std::string("_") +
                std::string(1, id.teamId)).build();
    }
    return RtDB2Context::Builder(id.shirtId)
            .withoutConfigFile()
            .withDatabase(dbname).build();
}
