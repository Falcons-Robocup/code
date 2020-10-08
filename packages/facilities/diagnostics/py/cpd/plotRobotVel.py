""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
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
rci.setRobotVelocity(0, 1, 0)
time.sleep(2)
rci.setRobotVelocity(0.0, 0.0, 0.0)
rci.setRobotVelocity(0, -1, 0)
time.sleep(2)
rci.setRobotVelocity(0.0, 0.0, 0.0)
rci.setRobotVelocity(1, 0, 0)
time.sleep(2)
rci.setRobotVelocity(0.0, 0.0, 0.0)
rci.setRobotVelocity(-1, 0, 0)
time.sleep(2)
rci.setRobotVelocity(0.0, 0.0, 0.0)
"""

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
