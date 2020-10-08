""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import os
import time
import urllib.request
import dateutil.parser

class BlocklyCMD:
    """
    This class executes a Blockly command on a (remote) robot.
    Can be applied to simulation with robot_hostname='localhost'
    """
    def __init__(self, cpd, robot_hostname, robot_id, blockly_cmd):

        self.cpd = cpd
        self.robot_hostname = robot_hostname
        self.robot_id = int(robot_id)
        self.blockly_cmd = blockly_cmd

    ####################
    # PUBLIC FUNCTIONS #
    ####################

    def executeBlocklyCommandOnRobot(self):
        """
        This method executes a Blockly command on a (remote) robot.
        Returns when the Blockly command has finished.
        Returns:
            newest_rdl_on_robot -- the filepath to the newest RDL file on the (remote) robot
            ageMin              -- the ageMin value, can be used to extract data from the returned RDL filename.
            ageMax              -- the ageMax value, can be used to extract data from the returned RDL filename.
        """

        # Get the RDL file location on the robot, and the timestamp it was created
        # /var/tmp/20200721_192718_r6.rdl
        # =>
        # datetime.datetime("2020-07-21 19:27:18")
        #
        # OR
        #
        # /var/tmp/20200721_192718.rdl
        # =>
        # datetime.datetime("2020-07-21 19:27:18")
        newest_rdl_on_robot = self.cpd.getNewestRDLFilenameFromRobot(self.robot_hostname)
        time_rdl_creation = dateutil.parser.parse( " ".join( os.path.basename(newest_rdl_on_robot).split(".")[0].split("_")[0:2] ) )

        # Determine "ageMin" for optimal parsing of the RDL
        time_before_action = self._getTimestampFromRobot()
        ageMin = (time_before_action - time_rdl_creation).seconds

        # Execute the move on the robot
        self._sendBlocklyCommandToRobot()

        # Wait for Blockly to finish
        self._waitUntilBlocklyFinishedOnRobot()

        # Determine "ageMax" for optimal parsing of the RDL
        time_after_action = self._getTimestampFromRobot()
        ageMax = (time_after_action - time_rdl_creation).seconds

        # Add extra margin to ageMin / ageMax to capture all data
        ageMin -= 1
        ageMax += 1

        return (newest_rdl_on_robot, ageMin, ageMax)

    #####################
    # PRIVATE FUNCTIONS #
    #####################

    def _sendBlocklyCommandToRobot(self):
        """
        Sends a Blockly command to the (remote) robot
        """

        data = urllib.parse.urlencode( {"robotNr": self.robot_id, "pythoncode": self.blockly_cmd} )
        data = data.encode('ascii')

        with urllib.request.urlopen("http://%s:8080/cmd" % (self.robot_hostname), data) as f:
            #print(f.read().decode('utf-8'))
            pass

    def _waitUntilBlocklyFinishedOnRobot(self):
        """
        Wait until the Blockly script has finished executing.
        """

        # Every 5 seconds, read MATCH_MODE from the (remote) robot.
        # If MATCH_MODE == True, Blockly has finished.
        blockly_finished = False
        while not blockly_finished:
            time.sleep(5)
            match_mode = self.cpd.doSshOnRobot(self.robot_hostname, "rget -a {0} MATCH_MODE".format(self.robot_id))

            if "value: True" in match_mode:
                blockly_finished = True

    def _getTimestampFromRobot(self):
        """
        Returns a datetime object of "now" on the (remote) robot
        """
        # $ ssh r3 date
        # =>
        # datetime.datetime("Tue Sep  1 19:33:37 CEST 2020")
        date_output = self.cpd.doSshOnRobot(self.robot_hostname, "date")
        return dateutil.parser.parse(date_output).replace(tzinfo=None)

