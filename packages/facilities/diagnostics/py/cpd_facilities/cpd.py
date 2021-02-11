# Copyright 2020 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
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
        #print(cmd)
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
        kst_binary = "kst2"
        cmd = "{} {}".format(kst_binary, plottable_filename)
        subprocess.call(cmd, shell=True)
