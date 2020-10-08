""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import os
import sys
import subprocess

import falconspy
import blocklycmd


main_cpd_data_dir = "/var/tmp/cpd"


class CPD:
    def __init__(self, cpd_name):

        self.cpd_name = cpd_name

        # Ensure the CPD data dir exists
        self.cpd_data_dir = os.path.join(main_cpd_data_dir, cpd_name)
        os.makedirs(self.cpd_data_dir, exist_ok=True)

        self.output_textfile = os.path.join(self.cpd_data_dir, self.cpd_name + ".txt")

    ####################
    # PUBLIC FUNCTIONS #
    ####################


    def executeBlocklyCommandOnRobot(self, robot_hostname, robot_id, blockly_cmd):
        """
        Execute Blockly command on (remote) robot)
        e.g., executeBlocklyCommandOnRobot("r3", 3, blockly_cmd)
        """

        blockly_cmd_obj = blocklycmd.BlocklyCMD(self, robot_hostname, robot_id, blockly_cmd)
        (newest_rdl_on_robot, ageMin, ageMax) = blockly_cmd_obj.executeBlocklyCommandOnRobot()
        return (newest_rdl_on_robot, ageMin, ageMax)


    def doSshOnRobot(self, robot_hostname, cmd):
        """
        Return the output of running 'cmd' on the robot via SSH.
        Takes care of sourcing the falcons environment.
        e.g., doSshOnRobot("r3", "python3 some_script.py")
        """

        # ssh -t robocup@localhost /bin/bash -ic \"rget -a 1 MATCH_MODE\"
        ssh_cmd = "ssh -t robocup@{0} /bin/bash -ic \\\"{1}\\\"".format(robot_hostname, cmd)
        #print(ssh_cmd)

        try:
            output = subprocess.check_output(ssh_cmd, stderr=subprocess.PIPE, shell=True).strip().decode()
        except subprocess.CalledProcessError as error:
            print("Failed to do SSH command '{0}' on hostname '{1}':".format(ssh_cmd, robot_hostname))
            print(error.output)
            exit()

        return output


    def getNewestRDLFilenameFromRobot(self, robot_hostname):
        """
        Returns the path to the newest RDL located on the (remote) robot
        e.g., getNewestRDLFilenameFromRobot("r3")
        """
        # $ ssh r3 $FALCONS_SCRIPTS_PATH/newestRDL
        # =>
        # /var/tmp/20200721_192718_r6.rdl
        return self.doSshOnRobot(robot_hostname, os.path.join(falconspy.FALCONS_SCRIPTS_PATH, "newestRDL"))


    def copyFileFromRobot(self, robot_hostname, filepath_on_robot):
        """
        Copies a file from the (remote) robot
        Returns the path to the file located on your laptop
        e.g., copyFileFromRobot("r3", "/path/to/file")
        """

        # Copy file from robot to self.cpd_data_dir
        # $ scp r3:/var/tmp/20200721_192718_r6.rdl self.cpd_data_dir
        cmd = "scp %s:%s %s" % (robot_hostname, filepath_on_robot, self.cpd_data_dir)
        subprocess.call(cmd, shell=True)

        # return self.cpd_data_dir/file.name
        return os.path.join( self.cpd_data_dir, os.path.basename(filepath_on_robot) )


    def parseDataFieldsFromRDLOnRobot(self, robot_hostname, robot_id, plot_data_enum, rdl_filename, ageMin, ageMax, output_textfile):
        """
        Reads the specified data fields from the RDL on the (remote) robot, and writes them in a plottable textfile on the (remote) robot.

        e.g., parseDataFieldsFromRDL("r3", 3, plotdata.PlotData.MOTION, "/var/tmp/20200827_205619_r2.rdl", ageMin, ageMax, output_textfile)
        """

        self.doSshOnRobot(robot_hostname, "python3 {}/plotfile.py --robotid {} --data {} --rdl_filename {} --ageMin {:06.2f} --ageMax {:06.2f} --output_filename {}".format(os.path.dirname(__file__), robot_id, plot_data_enum, rdl_filename, ageMin, ageMax, output_textfile))


    def plotData(self, plottable_filename):
        """
        Opens plottable_filename using KST
        e.g., plotData("/var/tmp/cpd/plotMoveX/plotMoveX.kst")
        """
        kst_binary = "/home/robocup/dev/kst/build/build/bin/kst2"
        cmd = "{} {}".format(kst_binary, plottable_filename)
        subprocess.call(cmd, shell=True)
