// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/plays/PlaySelection.hpp"
#include "int/plays/RoleAssignment.hpp"

#include "int/stores/GameStateStore.hpp"
#include "int/stores/RobotStore.hpp"

using namespace teamplay;

PlaySelection::PlaySelection()
{
    _currentPlay = PlayEnum::INVALID;
}

teamplay::PlayEnum PlaySelection::selectPlay()
{
    // TODO - introduce new plays

    // For now, default to our normal playstyle
    PlayEnum newPlay = PlayEnum::DEFAULT_PLAY;

    return newPlay;
}