""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
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

