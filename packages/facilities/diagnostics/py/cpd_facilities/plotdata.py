# Copyright 2020 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
import enum

class PlotData(enum.Enum):
    PLOT_ROBOT_VEL = 1,
    PLOT_ROBOT_POS = 2

plotDataMapping = {}

plotDataMapping[PlotData.PLOT_ROBOT_VEL] = \
{
    # m1.setpoint, m2.setpoint, m3.setpoint, m1.feedback, m2.feedback, m3.feedback
    "DIAG_PERIPHERALSINTERFACE": ["['speed_vel'][0]", "['speed_vel'][1]", "['speed_vel'][2]", "['feedback_vel'][0]", "['feedback_vel'][1]", "['feedback_vel'][2]"],

    # robot_vx.setpoint, robot_vy.setpoint, robot_vRz.setpoint
    "ROBOT_VELOCITY_SETPOINT": ["[0]", "[1]", "[2]"],

    # robot_vx.feedback, robot_vy.feedback, robot_vRz.feedback
    "ROBOT_VELOCITY_FEEDBACK": ["[0]", "[1]", "[2]"]
}

plotDataMapping[PlotData.PLOT_ROBOT_POS] = \
{
    "ROBOT_STATE": ["[2][0]", "[2][1]", "[2][2]"]
}
