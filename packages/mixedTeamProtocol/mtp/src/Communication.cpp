// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "int/Communication.hpp"

using namespace mtp;


Communication::Communication(PlayerId const &id, bool path_encoding)
:
    _id(id),
    _rtdb(id, "mixedteam", path_encoding),
    _rtdbRefbox(id, "refbox", path_encoding)
{
    // TODO: how to ensure current id is not already claimed? Rob? Should make requirement + test case in RTDB layer.

    // initialize RTDB state
    setState("CURRENT_ROLE", 0);
    setState("PREFERRED_ROLE", 0);
    setState("PREFERENCE_FACTOR", 0.0);
    setState("OWN_POS_VEL", mtp::PosVel());
    setState("HAS_BALL", false);
    setState("IS_LEADER", false);
    setState("GOOD", false);
}

Communication::~Communication()
{
}

std::vector<PlayerPacket> Communication::getPlayerPackets()
{
    std::vector<PlayerPacket> result;
    auto clients = _rtdb.getClients();
    for (auto& client: clients)
    {
        PlayerPacket packet;

        int r = _rtdb.get("MTP", &packet, client); // too slow?!!!
        if (r == RTDB2_SUCCESS) // TODO: revise RTDB API in v3 to not return magic int values, instead, apply exception handling
        {
            // same-team checks and timeout checks are done in Player packet handler
            result.push_back(packet);
        }
        /*else
        {
            if (r == RTDB2_ITEM_STALE)
            {
                RtDB2Item item;
                _rtdb.getItem("MTP", item);
                tprintf("WARNING: timeout (age %.2fs): could not read MTP packet for client %d at %s", item.age(), client, _id.describe().c_str());
            }
            // if only RTDB would just throw clear exceptions. TODO
        }*/
    }
    return result;
}

void Communication::setPlayerPacket(PlayerPacket const &packet)
{
    _rtdb.put("MTP", &packet);
}

std::string Communication::getFrameString()
{
    std::string result;

    RtDB2FrameSelection selection;
    selection.local = true;
    selection.shared = true;
    _rtdb.getFrameString(result, selection);
    return result;
}

void Communication::setFrameString(std::string const &s)
{
    _rtdb.putFrameString(s);
}

RefereeCommand Communication::getLastCommand()
{
    std::string command;
    std::string target;
    RefereeCommand::Arguments arguments;
    bool success = (_rtdbRefbox.get("COMMAND", &command, 0) == RTDB2_SUCCESS) &&
                   (_rtdbRefbox.get("TARGETTEAM", &target, 0) == RTDB2_SUCCESS) &&
                   (_rtdbRefbox.get("ARGUMENTS", &arguments, 0) == RTDB2_SUCCESS);
    RefereeCommand result;
    if (success)
    {
        result.command = commandStringToEnum(command);
        result.target = targetStringToEnum(target);
        result.arguments = arguments;
    }
    return result;
}

void Communication::startThread()
{
    if (_comm == NULL)
    {
        // frun rtdb comm/comm -a ${TURTLE5K_ROBOTNUMBER} -p /tmp/rtdb_mixedteam_A -n MTP
        // TODO: this needs simplifying. Too much configuration hassle, hacks, time lost each time.
        // TODO: also make configurable which threads comm should start. Use cases: 
        // 1. (human) client spoofing only needs transmitter, not receiver. 
        // 2. Diagnostics / visualizer on baseStation needs only receiver, not transmitter.
        auto b = RtDB2Context::Builder(_id.shirtId, RtDB2ProcessType::comm);
        b.withRootPath("/tmp/rtdb_mixedteam_A");
        b.withNetwork("MTP");
        RtDB2Context context = b.build();
        _comm = new Comm(context);
        _comm->settings.diagnostics = false;
        _comm->start();
    }
}

void Communication::stopThread()
{
    if (_comm != NULL)
    {
        _comm->shutdown();
        delete _comm;
        _comm = NULL;
    }
}
