# Copyright 2019-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
# load/manipulate a simulated scene


import sys, os
import argparse
import yaml

import falconspy
import sharedTypes
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH

DEFAULT_SCENE_FOLDER = falconspy.FALCONS_CODE_PATH + "/packages/simulation/scenes"


class TeleportBall():
    """
    Callback to modify the ball position and velocity.
    """
    def __init__(self, x, y, vx=0.0, vy=0.0):
        self.x = float(x)
        self.y = float(y)
        self.vx = float(vx)
        self.vy = float(vy)
    def __call__(self, scene):
        # set ball data
        scene["balls"] = [{"position": [self.x, self.y, 0.0], "velocity": [self.vx, self.vy, 0.0]}]
        # clear the robots and opponents, otherwise simulation will start teleporting robots etc.
        scene["robots"] = []
        scene["opponents"] = []
        # leave obstacles intact


class TeleportRobot():
    """
    Callback to modify friendly robot position.
    """
    def __init__(self, robotId, x, y, Rz=0.0, friendly=True):
        # allow user to specify robotId as int, but also as string like 'r4'
        if isinstance(robotId, str) and robotId[0] == "r":
            robotId = robotId[1:]
        self.robotId = int(robotId)
        self.x = float(x)
        self.y = float(y)
        self.Rz = float(Rz)
        self.friendly = friendly
    def __call__(self, scene):
        # clear the balls and opponents, otherwise simulation will start teleporting more than we want
        scene["balls"] = []
        scene["robots"] = []
        scene["opponents"] = []
        # set robot data
        targetList = "robots"
        if not self.friendly:
            targetList = "opponents"
        scene[targetList] = [{"robotId": self.robotId, "position": [self.x, self.y, self.Rz], "velocity": [0.0, 0.0, 0,0]}]
        # leave obstacles intact


class LoadYaml():
    """
    Callback to load a scene from a yaml file.
    """
    def __init__(self, yamlfile):
        # look in standard location if necessary
        if not os.path.isfile(yamlfile) and os.path.isfile(DEFAULT_SCENE_FOLDER + "/" + yamlfile):
            yamlfile = DEFAULT_SCENE_FOLDER + "/" + yamlfile
        self.yamlfile = yamlfile
    def __call__(self, scene):
        # load the yaml
        f = open(self.yamlfile, mode='r')
        y = yaml.load(f.read())
        f.close()
        # assign
        for key in scene.keys():
            scene[key] = y[key]


def validateScene(scene):
    # check that id's are increasing, starting at 1
    # this is currently a requirement by simulation
    for listsToCheck in ["robots", "opponents"]:
        robotList = scene[listsToCheck]
        ids = [int(r["robotId"]) for r in robotList]
        if (list(set(ids)) != sorted(ids)):
            print("ERROR: no duplicate robot id's allowed")
            sys.exit(1)


def run(modifier):
    """
    Given a callback, perform the following sequence:
    * RTDB get simulation scene
    * modify it using callback, validate
    * RTDB put simulation scene
    """

    # Create instance of RtDB2Store and read databases from disk
    path = RTDB2_DEFAULT_PATH
    agent = 0
    key = "SIMULATION_SCENE"
    rtdb2Store = RtDB2Store(path, False) # don't start in read-only

    # Get current value
    item = rtdb2Store.get(agent, key, timeout=False)

    # Modify value
    modifier(item.value)

    # Sanity checks
    validateScene(item.value)

    # Store and finish
    rtdb2Store.put(agent, key, item.value)
    rtdb2Store.closeAll()


if __name__ == '__main__':
    # Argument parsing.
    descriptionTxt = 'Manipulate the simulated scene. Will be used by simworld on-change.\n'
    exampleTxt = """Examples:
    simScene.py TCrun2.scene # to setup a TechChallenge
    simScene.py -b 1 0 0 -10 # to simulate a shot at goal keeper"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-b', '--ball', type=float, nargs=4, help='set ball position and velocity', metavar=('x', 'y', 'vx', 'vy')) # TODO: default v=0 if not provided?
    parser.add_argument('-r', '--robot', nargs=4, help='set friendly robot position', metavar=('id', 'x', 'y', 'Rz'))
    parser.add_argument('-o', '--opponent', nargs=4, help='set opponent robot position', metavar=('id', 'x', 'y', 'Rz'))
    parser.add_argument('yamlfile', help='yaml file to load', nargs='?')
    args       = parser.parse_args()

    # TODO: also allow json file?

    # run
    if args.yamlfile != None:
        modifier = LoadYaml(args.yamlfile)
    elif args.ball != None:
        modifier = TeleportBall(*args.ball)
    elif args.robot != None:
        modifier = TeleportRobot(*args.robot, friendly=True)
    elif args.opponent != None:
        modifier = TeleportRobot(*args.opponent, friendly=False)
    run(modifier)

