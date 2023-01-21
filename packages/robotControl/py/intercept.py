# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3

# wrapper script around the robotCLI intercept scenario

import sys
import rtdb2tools
import robotLibrary
import scenarios.intercept



if __name__ == '__main__':
    args = scenarios.intercept._parse_arguments(sys.argv[1:])
    if args.robot == None:
        args.robot = rtdb2tools.guessAgentId()
        if args.robot == None or args.robot == 0:
            raise RuntimeError("could not determine robot ID, either this script should run on a robot, or it must be explicitly specified as argument")
    # setup robotlibrary
    robot = robotLibrary.RobotLibrary(args.robot)
    # run the scenario
    interceptor = scenarios.intercept._Interceptor(robot, args)
    interceptor.run() # TODO: robust for ctrl-C?
    # shutdown
    robot.shutdown()

