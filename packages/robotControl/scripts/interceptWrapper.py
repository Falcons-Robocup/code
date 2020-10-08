""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3

# Jan Feitsma, February 2020

# Let multiple robots perform the intercept behavior, by calling remotely intercept.py per robot.
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
    parser = argparse.ArgumentParser(description="Let multiple robots perform the intercept action in a circle. Wrapper around intercept.py.")
    parser.add_argument('-c', '--circleradius', help='home position circle radius on which robot default positions are set', type=float, default=4.0)
    parser.add_argument('-n', '--targetnoise', help='aim given amount of meters at a random side next to the target', type=float, default=0.0)
    parser.add_argument('--mode', help='sim or real mode, guess if not provided', default=None, choices=['sim', 'real'])
    parser.add_argument("-r", "--robots", type=str, default="r2,r3,r4,r5,r6,r7", help="comma separated list of robots, for example: -r r2,r6,r3")
    parser.add_argument('-f', '--force', help='disable standard checks for robots being active', action='store_true')
    return parser.parse_args()


def isAlive(hostname):
    command = "ping -W 1 -c 1 {0} >/dev/null 2>/dev/null".format(hostname)
    return (os.system(command) == 0)


def isActive(robot):
    item = rtdb2Store.get(robot, "ROBOT_STATE")
    return item != None


def runScript(robot, command, remote=False, cleanup=True):
    # optionally wrap the command to enable remote execution
    envcommand = "source /home/robocup/falcons/code/scripts/setupEnv.sh"
    if remote:
        command = "ssh {0} \"{1} ; {2}\"".format(robot, envcommand, command)
    try:
        result = os.system(command)
    except e:
        import traceback
        traceback.print_exc()
        pass
    if cleanup and remote:
        # prevent robots continuing due after ssh connection closure
        command = "killRobotCLI"
        command = "ssh {0} \"{1} ; {2}\"".format(robot, envcommand, command)
        os.system(command)



if __name__ == '__main__':
    args = parse_arguments()
    robots = list()

    # guess mode?
    if args.mode == None:
        if socket.gethostname() == "coach":
            args.mode = "real"
        else:
            # quick check if a simulation is running
            item = rtdb2Store.get(1, "TP_HEARTBEAT") # making use of the fact that key is configured to be local
            if item != None:
                args.mode = "sim"
    if args.mode not in ["sim", "real"]:
        raise RuntimeError("Failed to guess 'mode', please use the option")

    # determine list of robots
    if args.mode == "real":
        for r in args.robots.split(','):
            if args.force or (isAlive(r) and isActive(int(r[1]))):
                robots.append(r)
    elif args.mode == "sim":
        possible_simulated_robots = ["r2", "r3", "r4", "r5"] # r1 cannot participate
        for r in args.robots.split(','):
            if r in possible_simulated_robots:
                if args.force or isActive(r[1]):
                    robots.append(r)

    # no robots online?
    if len(robots) == 0:
        raise RuntimeError("All robots and/or coachControl are offline (use option '--force'?)")

    # execute
    remote = (args.mode == "real")
    INTERCEPT_SCRIPT = "/home/robocup/falcons/code/packages/robotControl/scripts/intercept.py"
    if len(robots) == 1:
        robot = robots[0]
        # special solo behavior: pass into the goal
        command = INTERCEPT_SCRIPT + " -r {} --home 0 5 -w --target 0 16 --targetnoise={}".format(robot[1], args.targetnoise)
        runScript(robot, command, remote)
    else:
        for robot in robots:
            # use threading to run the blocking commands in parallel
            command = INTERCEPT_SCRIPT + " -q -r {} --circleradius={} --actionradius={} --targetnoise={}".format(robot[1], args.circleradius, args.circleradius*0.5, args.targetnoise)
            t = threading.Thread(target=runScript, args=(robot, command, remote))
            t.start()

