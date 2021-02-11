# Copyright 2020 Erik Kouters (Falcons)
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

blockly_cmd = """
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).id})
rci.blockUntilTPOverridePassedOrFailed()
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_LEFT )).id})
rci.blockUntilTPOverridePassedOrFailed()
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OWN_PENALTY_SPOT )).id})
rci.blockUntilTPOverridePassedOrFailed()
rci.setTeamplayAction("MOVE", {"target": (EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_CIRCLE_INTERSECT_LINE_RIGHT )).id})
rci.blockUntilTPOverridePassedOrFailed()
"""

class PlotRobotPos(cpd.CPD):

    def __init__(self):
        super(PlotRobotPos, self).__init__("plotRobotPos")


    def execute(self, robot_hostname, robot_id):

        # Execute Blockly command on (remote) robot
        (newest_rdl_on_robot, ageMin, ageMax) = self.executeBlocklyCommandOnRobot(robot_hostname, robot_id, blockly_cmd)

        # Parse RDL into plottable textfile on (remote) robot
        self.parseDataFieldsFromRDLOnRobot(robot_hostname, robot_id, plotdata.PlotData.PLOT_ROBOT_POS, newest_rdl_on_robot, ageMin, ageMax, self.output_textfile)

        # Copy the plottable textfile from the robot
        plottable_textfile_local = self.copyFileFromRobot(robot_hostname, self.output_textfile)

        # Open KST2 plot
        kst_file = os.path.join( os.path.dirname(__file__), "plotRobotPos.kst")
        self.plotData( kst_file )



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script executes a move on a robot, and plots the result using KST")
    parser.add_argument("--robot_hostname", type=str, required=True, help="the robot hostname to execute the move on. Example: --robot_hostname r3")
    parser.add_argument("--robot_id", type=int, required=True, help="the robot id to execute the move on. Example: --robot_id 3")
    args = parser.parse_args()

    plotRobotPos = PlotRobotPos()
    plotRobotPos.execute( args.robot_hostname, args.robot_id )
