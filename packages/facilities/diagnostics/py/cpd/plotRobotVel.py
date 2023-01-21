# Copyright 2020-2021 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#

import argparse
import urllib.request
import os
import subprocess
import time
import dateutil.parser

import falconspy
import rdlLib
import cpd
import plotdata

### vy =  1.0 for 2 seconds
### vRz = 2.0 for 2 seconds
### vx =  1.0 for 2 seconds
blockly_cmd_velocity = """
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, "NORMAL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, "NORMAL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, "NORMAL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(2)
"""

### Go to (0,0)
### Drive forward/backward (no turning)
### Drive sideways (no turning)
### Drive diagonal (no turning)
### Drive forward/backward while turning around
### Drive sideways while turning around
### Drive diagonal while turning around
blockly_cmd_position = """
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)

rci.setRobotPosVel("POS_ONLY", {distance}, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)

rci.setRobotPosVel("POS_ONLY", 0.0, {distance}, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)

rci.setRobotPosVel("POS_ONLY", {distance}, {distance}, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)

rci.setRobotPosVel("POS_ONLY", {distance}, 0.0, 3.14, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)

rci.setRobotPosVel("POS_ONLY", 0.0, {distance}, 3.14, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)

rci.setRobotPosVel("POS_ONLY", {distance}, {distance}, 3.14, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "NORMAL")
time.sleep(4)
""".format(distance='3.0')


blockly_cmd_position_2 = """
rci.setMotionPlanningAction("MOVE", (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).x, (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).y, 0.0, "NORMAL", True)
rci.blockUntilMPPassedOrFailed()
rci.setMotionPlanningAction("MOVE", (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_LEFT )).x, (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_LEFT )).y, 3.14, "NORMAL", True)
rci.blockUntilMPPassedOrFailed()
rci.setMotionPlanningAction("MOVE", (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OWN_PENALTY_SPOT )).x, (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OWN_PENALTY_SPOT )).y, 0.0, "NORMAL", True)
rci.blockUntilMPPassedOrFailed()
rci.setMotionPlanningAction("MOVE", (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).x, (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).y, 3.14, "NORMAL", True)
rci.blockUntilMPPassedOrFailed()
"""

blockly_cmd_bh = """
rci.setBallHandlers(True)
time.sleep(5)
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).id})
rci.blockUntilTPOverridePassedOrFailed()
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_LEFT )).id})
rci.blockUntilTPOverridePassedOrFailed()
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OWN_PENALTY_SPOT )).id})
rci.blockUntilTPOverridePassedOrFailed()
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).id})
rci.blockUntilTPOverridePassedOrFailed()
"""

blockly_cmd_bh_2 = """
rci.setBallHandlers(True)
time.sleep(5)
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).id})
rci.blockUntilTPOverridePassedOrFailed()
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CENTER )).id})
rci.blockUntilTPOverridePassedOrFailed()
"""

blockly_cmd_bh_y_pos = """
rci.setBallHandlers(True)
time.sleep(5)
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(2)
rci.setRobotPosVel("POS_ONLY", 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(4)
rci.setRobotPosVel("POS_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(4)
"""

blockly_cmd_bh_y = """
rci.setBallHandlers(True)
time.sleep(5)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 1.2, 0.0, "WITH_BALL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, -0.5, 0.0, "WITH_BALL")
time.sleep(4)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(2)
"""

blockly_cmd_bh_x = """
rci.setBallHandlers(True)
time.sleep(5)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, "WITH_BALL")
time.sleep(3)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, "WITH_BALL")
time.sleep(3)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(2)
"""

blockly_cmd_bh_rz = """
rci.setBallHandlers(True)
time.sleep(5)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, "WITH_BALL")
time.sleep(3)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(2)
"""

blockly_cmd_bh_x_y_rz = """
rci.setBallHandlers(True)
time.sleep(5)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, -0.5, -0.5, -1.0, "WITH_BALL")
time.sleep(3)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(2)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.5, 1.2, 1.0, "WITH_BALL")
time.sleep(3)
rci.setRobotPosVel("VEL_ONLY", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "WITH_BALL")
time.sleep(2)
"""

### Choose blockly_scenario to execute here
blockly_cmd = blockly_cmd_position_2


class PlotRobotVel(cpd.CPD):

    def __init__(self):
        super(PlotRobotVel, self).__init__("plotRobotVel")


    def execute(self, robot_hostname, robot_id):

        # Execute Blockly command on (remote) robot
        (newest_rdl_on_robot, ageMin, ageMax) = self.executeBlocklyCommandOnRobot(robot_hostname, robot_id, blockly_cmd)

        # Parse RDL into plottable textfile on (remote) robot
        self.parseDataFieldsFromRDLOnRobot(robot_hostname, robot_id, plotdata.PlotData.PLOT_ROBOT_VEL, newest_rdl_on_robot, ageMin, ageMax, self.output_textfile)

        # Copy the plottable textfile from the robot
        plottable_textfile_local = self.copyFileFromRobot(robot_hostname, self.output_textfile)

        # Open KST2 plot
        kst_file = os.path.join( os.path.dirname(__file__), "plotRobotVel.kst")
        self.plotData( kst_file )



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script executes a move on a robot, and plots the result using KST")
    parser.add_argument("--robot_hostname", type=str, required=True, help="the robot hostname to execute the move on. Example: --robot_hostname r3")
    parser.add_argument("--robot_id", type=int, required=True, help="the robot id to execute the move on. Example: --robot_id 3")
    args = parser.parse_args()

    plotRobotVel = PlotRobotVel()
    plotRobotVel.execute( args.robot_hostname, args.robot_id )
