// Copyright 2016-2022 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef TEAMROBOTSELECTION_H
#define TEAMROBOTSELECTION_H

#include <cstdint>
#include <variant>

class TeamRobotSelection
{
public:
    void setTeamMode();
    void setRobotMode(uint8_t id);

protected:
    enum class TEAM
    {
        OWN
    };
    using RobotId = uint8_t;

    std::variant<TEAM, RobotId> _viewMode{TEAM::OWN};

    virtual void onTeamModeChanged() {};
    virtual void onRobotModeChanged() {};
};

#endif // TEAMROBOTSELECTION_H
