""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Python command-line interface for commands, on-robot.
# Intended for integrators and developers.
# Examples:
#    robotCLI.py     # for an interactive prompt
#    robotCLI.py target 0 0 0
#    robotCLI.py scenario interceptBall
#    robotCLI.py scenario kpiBhSpeedX
#
# Implementations:
#    robotCLI.py          # command-line interface
#    robotLibrary.py      # discloses all basic commands and scenarios, also includes parser
#    robotScenarios.py    # scenario interface
#    robotRosInterface.py # basic commands and their ROS interfaces
#    scenarios/*.py       # scenario implementations
#
# Jan Feitsma, 2016-12-17



import sys
import argparse
import robotLibrary

import logging
logging.basicConfig()

if __name__ == '__main__':

    # argument parsing
    parser     = argparse.ArgumentParser(description='robot command interface')
    parser.add_argument('--robotId', '-r', help='robot number', default=None, type=int)
    parser.add_argument('--verbose', '-v', help='verbose', action='store_true')
    parser.add_argument('--extraverbose', '-vv', help='extra verbose', action='store_true')
    args, leftovers = parser.parse_known_args()
    
    # which robot to influence?
    if args.robotId == None:
        robotLibrary.guessRobotId()
    else:
        robotLibrary.setRobotId(args.robotId)
        
    # initialize
    robotLibrary.initialize()
    if args.verbose:
        robotLibrary.setVerbose(1)
    if args.extraverbose:
        robotLibrary.setVerbose(2)
    
    # has a command been given? if not, provide a prompt
    if len(leftovers):
        commandStr = ' '.join(leftovers)
        robotLibrary.parse(commandStr)
    else:
        robotLibrary.prompt()
        
    # done
    try: # TODO: when giving a ctrl-C, it seems the service to teamplay is not valid anymore ... ?!
        robotLibrary.shutdown()
    except:
        pass
    sys.exit(0)
        
