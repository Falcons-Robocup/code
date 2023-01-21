// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "ext/MixedTeamProtocol.hpp"

// other headers from this package
#include "int/MixedTeamProtocolImpl.hpp"


using namespace mtp;

MixedTeamProtocol::MixedTeamProtocol(PlayerId const &id, bool path_encoding)
    : impl(std::make_unique<MixedTeamProtocolImpl>(id, path_encoding))
{
}
