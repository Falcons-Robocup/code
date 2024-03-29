# Copyright 2020-2021 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
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
import falconsrtdb


# setup RTDB
rtdbStore = falconsrtdb.FalconsRtDBStore(readonly=False) # write mode



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
    item = rtdbStore.get(robot, "ROBOT_STATE")
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
    rtdbStore.put(0, "REFBOX_OVERRIDE", {"command": "PARK"})


if __name__ == '__main__':
    config = parse_arguments()
    robots = list()

    # guess mode?
    if config.mode == None:
        if socket.gethostname() == "coach":
            config.mode = "real"
        else:
            # quick check if a simulation is running
            item = rtdbStore.get(1, "TP_HEARTBEAT") # making use of the fact that key is configured to be local
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

