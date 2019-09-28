""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Python library for on-robot commands.
#
# Implementations:
#    robotCLI.py          # command-line interface
#    robotLibrary.py      # discloses all basic commands and scenarios, also includes parser
#    robotScenarios.py    # scenario interface
#    robotInterface.py    # basic commands and their ROS interfaces
#    scenarios/*.py       # scenario implementations
#
# Jan Feitsma, 2016-12-17



import sys, os, signal
import traceback
import logging
import readline
from time import sleep

import robotInterface as robot
from robotScenarios import *


# globals
_robotId = None
_initialized = False
logging.basicConfig(level=logging.INFO)


def initialize():
    """
    Setup the robot interface.
    """
    global _initialized
    assert(_initialized == False)
    initializeLogger()
    robot.connect(_robotId)
    _initialized = True
    # on robot: initialize BH
    robot.enableBallHandlers()

def initializeLogger():
    """
    Setup the logging environment
    """
    log = logging.getLogger()  # root logger
    log.setLevel(logging.INFO)
    format_str = '%(asctime)s.%(msecs)03d - %(levelname)-8s - %(message)s'
    date_format = '%Y-%m-%dT%H:%M:%S'
    formatter = logging.Formatter(format_str, date_format)
    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(formatter)
    log.handlers = [] # clear
    log.addHandler(stream_handler)
    return logging.getLogger(__name__)

def shutdown():
    """
    Cleanup such that robot is in a good state to play (refbox).
    """
    # make all threads stop, see e.g. ballMonitor
    robot.disconnect()

def guessRobotId():
    """
    Guess the robot to target.
    If used on real robot, this will find the robot namespace.
    """
    robotId = os.getenv("TURTLE5K_ROBOTNUMBER")
    if robotId != None:
        setRobotId(int(robotId))
    # TODO: simulation use case: in case we only have one simulated robot active, it would be nice if user does not have to specify the robot explicitly

def setRobotId(robotId):
    """
    Bind this instance to a robot.
    """
    assert(robotId in range(1, 20))
    global _robotId
    _robotId = robotId

def prompt():
    """
    Provide a prompt to users, so they can enter commands.
    """
    logging.info("(enter h for help)")
    while True:
        try:
            # TODO: prepend with timestamp, nicely align with logging?
            s = raw_input("enter command: ")
        except:
            logging.error("error while reading input")
            break
        if s in ["q", "quit", "shutdown"]:
            break
        try:
            parse(s)
        except:
            logging.warning("something went wrong while evaluating '%s'" % (s))
            traceback.print_exc(file=sys.stdout)
    logging.info("shutting down ...")
    shutdown()

def printHelp():
    print helpText()

def helpText():
    return """
most commands are blocking, which means you cannot type anything until it finished
(TODO: implement ctrl-C to interrupt such blocking commands and return back to the prompt)
COMMANDS:
    h / help / ?           : show this help
    q / quit / shutdown    : exit program
    ctrl-C                 : interrupt running command (TODO)
    stop                   : stop whatever robot is doing
    sleep s                : sleep for s seconds [blocking]
    bh on|off              : enable/disable ball handlers
    velocity vx vy vphi s  : give a relative velocity setpoint (vx,vy,vphi) for s seconds
    move x y phi           : move robot to (x,y,phi) using pathPlanning
    getBall                : get the ball
    kick power height      : kick ball with given power (30..180) and optional height (0..180)
    pass x y               : pass to (x,y)
    shoot x y [z]          : straight shot at (x,y,z)
    lob x y [z]            : lob shot at (x,y,z)
    keeper                 : start acting as keeper
    behavior name          : perform a behavior
    listScenarios          : shows a list of loaded scenarios
    scenario name          : run a pre-defined scenario [blocking]
"""
# next ones still need to be fixed after migration to RTDB! #62
# play                   : change behavior to neutral_playing"
#print "  action name [args]     : perform an action [not blocking]"
#print "  actionBlocking name [args] : perform an action [blocking]"

def parse(commandString):
    """
    Parse given command string.
    """
    logging.info("parsing command: '%s'" % (commandString))
    assert(_initialized == True)
    # split string
    words = commandString.split()
    if len(words) == 0:
        return
    mode = words[0].lower()
    args = words[1:]
    # determine what to do, catch interrupts
    try:
        if mode in ['h', 'help', '?']:
            printHelp()
        elif mode in ['stop']:
            robot.stop()
        elif mode in ['sleep']:
            sleep(float(args[0]))
        elif mode in ['bh']:
            if len(args) == 1:
                if str(args[0]) == "on":
                    robot.enableBallHandlers()
                if str(args[0]) == "off":
                    robot.disableBallHandlers()
        elif mode in ['velocity']:
            vx = float(args[0])
            vy = float(args[1])
            vphi = float(args[2])
            t = float(args[3])
            robot.setVelocity(vx, vy, vphi, t)
        elif mode in ['move', 'target']:
            x = float(args[0])
            y = float(args[1])
            # phi not required, default to zero
            if len(args) > 2:
                phi = float(args[2])
            else:
                phi = 0
            robot.move(x, y, phi)
        elif mode in ['getball']:
            robot.getBall()
        elif mode in ['pass']:
            x = float(args[0])
            y = float(args[1])
            robot.passTo(x, y)
        elif mode in ['shoot']:
            x = float(args[0])
            y = float(args[1])
            # z not required, default to zero
            if len(args) > 2:
                z = float(args[2])
            else:
                z = 0
            robot.shootAt(x, y, z)
        elif mode in ['lob']:
            x = float(args[0])
            y = float(args[1])
            # z not required, default to zero
            if len(args) > 2:
                z = float(args[2])
            else:
                z = 0
            robot.lobShotAt(x, y, z)
        elif mode in ['kick']:
            power = float(args[0])
            height = 0
            if len(args) > 1:
                height = float(args[1])
            robot.kick(power, height)
        elif mode in ['keeper']:
            logging.warning("known issue: this will continue forever, until you ctrl-C / restart robotCLI!")
            robot.behavior("B_GOALKEEPER")
        elif mode in ['behavior']:
            robot.behavior(args[0])
        # TODO: stimulate teamplay on more levels, e.g. gameState neutralPlaying
        elif mode in ['listscenarios']:
            print "Loaded scenarios are:"
            print listScenarios()
            print "\n To execute, type scenario <name>"
        elif mode in ['scenario']:
            # If no arguments given, prompt with a selection menu
            if len(args) == 0:
                # No argument given, prompt for the scenarios
                scenarios = listScenarios()
                scenarios.sort()
                idx = 1
                print "Scenarios available for execution:"
                print "0: Cancel"
                for scenario in scenarios:
                    print "%s: %s" % (idx, scenario)
                    idx += 1
                userinput = input("Choose a scenario to run: ")
                if isinstance( userinput, ( int, long ) ):
                    if int(userinput) > 0:
                        runScenario( scenarios[int(userinput) - 1] ) # TODO Add support for arguments when prompting selection menu
                        robot.restoreAllConfigs()
            else:
                # relay to scenario selector/executor
                runScenario(*args)
                # After running a scenario, restore all dynamic_reconfigure configs
                # We do this to reset the state of the robot after every scenario
                robot.restoreAllConfigs()
        else:
            logging.error("invalid or not-yet-implemented mode '%s'" % (mode))
        """
        # TODO: redesign monitors
        elif mode in ['analyzeball']:
            monitorBall()
        elif mode in ['analyzeloc']:
            monitorLocalization()
        """
        """TODO
        except KeyboardInterruptException:
            # probably a blocking call was interrupted, ok to continue (if prompted)
            logging.warning("--interrupted--")
            return
        """
    except:
        logging.warning("something went wrong while evaluating '%s'" % (commandString))
        traceback.print_exc(file=sys.stdout)



