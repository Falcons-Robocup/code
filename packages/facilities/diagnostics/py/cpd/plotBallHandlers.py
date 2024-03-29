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

### vy =  1.0 for 2 seconds
### vy = -1.0 for 2 seconds
blockly_cmd = """
rci.setBallHandlers(True)
time.sleep(5)
"""

class PlotBallHandlers(cpd.CPD):

    def __init__(self):
        super(PlotBallHandlers, self).__init__("plotBallHandlers")


    def execute(self, robot_hostname, robot_id):

        # Execute Blockly command on (remote) robot
        (newest_rdl_on_robot, ageMin, ageMax) = self.executeBlocklyCommandOnRobot(robot_hostname, robot_id, blockly_cmd)

        # Parse RDL into plottable textfile on (remote) robot
        self.parseDataFieldsFromRDLOnRobot(robot_hostname, robot_id, plotdata.PlotData.PLOT_BH, newest_rdl_on_robot, ageMin, ageMax, self.output_textfile)

        # Copy the plottable textfile from the robot
        plottable_textfile_local = self.copyFileFromRobot(robot_hostname, self.output_textfile)

        # Open KST2 plot
        kst_file = os.path.join( os.path.dirname(__file__), "plotBallHandlers.kst")
        self.plotData( kst_file )



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script executes a move on a robot, and plots the result using KST")
    parser.add_argument("--robot_hostname", type=str, required=True, help="the robot hostname to execute the move on. Example: --robot_hostname r3")
    parser.add_argument("--robot_id", type=int, required=True, help="the robot id to execute the move on. Example: --robot_id 3")
    args = parser.parse_args()

    if args.robot_hostname == "localhost":
        print("This CPD does not support simulation. Exiting...")
        exit()

    plotBallHandlers = PlotBallHandlers()
    plotBallHandlers.execute( args.robot_hostname, args.robot_id )
