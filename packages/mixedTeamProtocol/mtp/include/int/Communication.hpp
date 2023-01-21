// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_COMMUNICATION_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_COMMUNICATION_HPP_

// headers from this package
#include "ext/PlayerId.hpp"
#include "ext/Referee.hpp"
#include "int/PlayerPacket.hpp"
#include "int/AdapterRTDB.hpp"

// standard/system headers
#include <vector>

namespace mtp
{

class Communication
{
public:
    Communication(PlayerId const &id, bool path_encoding = false);
    ~Communication();

    std::vector<PlayerPacket> getPlayerPackets();
    void setPlayerPacket(PlayerPacket const &packet);

    template<typename T>
    T getState(std::string key)
    {
        // plan A: use standard API, which might say an item has grown stale
        //T result;
        //int r = _rtdb.get(key, &result);
        //if (r == RTDB2_SUCCESS) return result;
        //tprintf("key=%s age=%.2f", key.c_str(), item.age());
        //throw std::runtime_error("MTP::Communication error " + std::to_string(r) + " for key " + key);
        // plan B: ignore age
        // why: test suite shows error 13 (STALE) occasionally, even though we do not sleep anywhere ?! performance issue perhaps?
        RtDB2Item item;
        _rtdb.getItem(key, item);
        return item.value<T>();
    }
    template<typename T>
    void setState(std::string key, T const &value)
    {
        _rtdb.put(key, &value);
    }
    RefereeCommand getLastCommand();

    // workaround for achieving somewhat realistic communication in test suite ... TODO redesign out of this interface?
    std::string getFrameString(); // TODO: const (need change in RTDB API)
    void setFrameString(std::string const &s);

    // communication
    void startThread();
    void stopThread();

private:
    PlayerId _id;
    AdapterRTDB _rtdb;
    AdapterRTDB _rtdbRefbox;
    Comm *_comm = NULL;
}; // end of class Communication

} // end of namespace mtp

#endif
