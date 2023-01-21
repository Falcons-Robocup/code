// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "ext/MTPAdapter.hpp"

// other headers from this package
#include "int/MTPAdapterImpl.hpp"

// headers from other packages
#include "falconsCommonEnv.hpp" // for getRobotNumber



mtp::PlayerId createPlayerId(char team)
{
    return createPlayerId(getRobotNumber(), team);
}

mtp::PlayerId createPlayerId(int id, char team)
{
    int FALCONS = 74; // TODO: need enum in MTP or a more common 'TeamInfo' repo?
    return mtp::PlayerId(FALCONS, id, team);
}

MixedTeamProtocolAdapter::MixedTeamProtocolAdapter()
    : impl(std::make_unique<MixedTeamProtocolAdapterImpl>(createPlayerId(getTeamChar())))
{
}

MixedTeamProtocolAdapter::MixedTeamProtocolAdapter(mtp::PlayerId const &id)
    : impl(std::make_unique<MixedTeamProtocolAdapterImpl>(id))
{
}
