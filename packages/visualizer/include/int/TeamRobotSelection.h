// Copyright 2016-2019 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef TEAMROBOTSELECTION_H
#define TEAMROBOTSELECTION_H

#include <cstdint>

class TeamRobotSelection
{
public:
    void setTeamMode();
    void setRobotMode(uint8_t id);

protected:
    enum ViewMode
    {
        TEAM,
        ROBOT
    };
    ViewMode _viewMode = TEAM;
    uint8_t _robotModeId = 0; // default team

    virtual void onTeamModeChanged() {};
    virtual void onRobotModeChanged() {};
};

#endif // TEAMROBOTSELECTION_H
