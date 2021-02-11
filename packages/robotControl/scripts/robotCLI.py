# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
# Python command-line interface for commands, on-robot.


import os
import sys
import argparse
import robotLibrary



def guessRobotId():
    """
    Guess the robot to target.
    If used on real robot, this will find the robot namespace.
    Will return None otherwise
    """
    robotId = os.getenv("TURTLE5K_ROBOTNUMBER")
    if robotId != None:
        return int(robotId)
    else:
        return None

if __name__ == '__main__':

    # argument parsing
    parser     = argparse.ArgumentParser(description='robot command interface', epilog=robotLibrary.helpText(), formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robotId', '-r', help='robot number', default=None, type=int)
    parser.add_argument('--verbose', '-v', help='verbose', action='store_true')
    parser.add_argument('--extraverbose', '-vv', help='extra verbose', action='store_true')
    parser.add_argument('command', nargs='?', help='command string to execute', default=None)
    args, leftovers = parser.parse_known_args()

    # If no robotId given, obtain the Id from environment variable (real robot only)
    if args.robotId == None:
        robotId = guessRobotId()
    else:
        robotId = args.robotId

    # Instantiate and initialize RobotLibrary
    robotLib = robotLibrary.RobotLibrary(robotId)

    if args.verbose:
        robotLib.setVerbose(1)
    if args.extraverbose:
        robotLib.setVerbose(2)
    
    # has a command been given? if not, provide a prompt
    if args.command != None:
        commandStr = args.command + " " + ' '.join(leftovers)
        robotLib.parse(commandStr)
    else:

        robotLib.prompt()
    
    # done
    try:
        robotLib.shutdown()
    except:
        pass
    sys.exit(0)

