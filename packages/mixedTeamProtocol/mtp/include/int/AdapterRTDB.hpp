// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_ADAPTERRTDB_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_ADAPTERRTDB_HPP_

// headers from external dependencies
#include "RtDB2.h"
#include "Player.hpp"
#include "comm.hpp"

#define MTP_RTDB_STORAGE_PATH "/tmp/rtdb_"

// standard/system headers
#include <vector>

namespace mtp
{

class AdapterRTDB: public RtDB2
{
public:
    AdapterRTDB(PlayerId const &id, std::string const &dbname = "default", bool path_encoding = false);
    ~AdapterRTDB();

    std::set<int> getClients(); // TODO const; - require some fundamental rework in RTDB --> v3?

private:
    RtDB2Context createContext(PlayerId const &id, std::string const &dbname, bool path_encoding);

}; // end of class AdapterRTDB

} // end of namespace mtp

#endif
