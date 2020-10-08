""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3

# Jan Feitsma, February 2020

# Park all robots.
# For description and usage hints, execute with '-h'


import argparse
import sys, os
import time
import threading
import socket # for gethostname
import falconspy
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH


# setup RTDB
rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH, False)



def parse_arguments():
    parser = argparse.ArgumentParser(description="Park all robots. Works for real robots as well as for simulation.")
    parser.add_argument('--refbox', help='use refbox signal', action='store_true')
    parser.add_argument('--mode', help='sim or real mode, guess if not provided', default=None, choices=['sim', 'real'])
    parser.add_argument("-r", "--robots", type=str, default="r1,r2,r3,r4,r5,r6,r7", help="comma separated list of robots, for example: -r r1,r6,r3")
    return parser.parse_args()


def isAlive(hostname):
    command = "ping -W 1 -c 1 {0} >/dev/null 2>/dev/null".format(hostname)
    return (os.system(command) == 0)


def isActive(robot):
    item = rtdb2Store.get(robot, "ROBOT_STATE")
    return item != None


def parkUsingRobotCLI(robot, role, remote=False):
    command = "robotCLI.py -r {0} behavior PARK role={1}".format(robot[1], role)
    # wrap the command to enable remote execution?
    if remote:
        envcommand = "source /home/robocup/falcons/code/scripts/setupEnv.sh"
        command = "ssh {0} \"{1} ; {2}\"".format(robot, envcommand, command)
    result = os.system(command)
    print("done "+robot)
    if result != 0:
        raise RuntimeError("Failed to run command: {1}".format(robot, command))


def parkUsingRefbox():
    rtdb2Store.put(0, "REFBOX_OVERRIDE", {"command": "PARK"})


if __name__ == '__main__':
    config = parse_arguments()
    robots = list()

    # guess mode?
    if config.mode == None:
        if socket.gethostname() == "coach":
            config.mode = "real"
        else:
            # quick check if a simulation is running
            item = rtdb2Store.get(1, "TP_HEARTBEAT") # making use of the fact that key is configured to be local
            if item != None:
                config.mode = "sim"
    if config.mode not in ["sim", "real"]:
        raise RuntimeError("Failed to guess 'mode', please use the option")

    # determine list of robots
    if config.mode == "real":
        for r in config.robots.split(','):
            if isAlive(r) and isActive(r[1]):
                robots.append(r)
    elif config.mode == "sim":
        possible_simulated_robots = ["r1", "r2", "r3", "r4", "r5"]
        for r in config.robots.split(','):
            if r in possible_simulated_robots:
                if isActive(r[1]):
                    robots.append(r)

    # a list of roles to be used, which map to unique target positions
    # TODO: it would be nice if teamplay allows omitting this explicit context
    # (can't teamplay use latest role in memory?)
    roles = ['R_goalkeeper', 'attackerAssist', 'defenderAssist', 'attackerMain', 'defenderMain']

    # execute
    remote = (config.mode == "real")
    if config.refbox:
        parkUsingRefbox()
    else:
        roleIdx = 0
        for robot in robots:
            role = roles[roleIdx]
            if roleIdx >= 5:
                print("WARNING: no more roles/spots available, there are too many robots online -- skipping robot " + robot)
            else:
                # use threading to run the blocking commands in parallel
                t = threading.Thread(target=parkUsingRobotCLI, args=(robot, role, remote))
                t.start()
            roleIdx += 1

