""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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
#    robotRosInterface.py # basic commands and their ROS interfaces
#    scenarios/*.py       # scenario implementations
# 
# Jan Feitsma, 2016-12-17



import sys, os, signal
import traceback
from robotActions import *
from robotScenarios import *
import readline


# globals
_robotId = None
_initialized = False
_verbose = False


def initialize():
    """
    Setup the ROS interface.
    """
    global _initialized
    assert(_initialized == False)
    #sys.stdout.write("initializing ...")
    #sys.stdout.flush()
    constructRosInterface(_robotId)
    _initialized = True
    # on robot: initialize BH
    stop() # needed to disable teamplay by default, otherwise ballHandlers are actively disabled
    if not isSimulated():
        enableBallHandlers()
    #print " done"

def shutdown():
    """
    Cleanup such that robot is in a good state to play (refbox).
    """
    tpControl("reset")
    restoreAllConfigs()
    
def guessRobotId():
    """
    Guess the robot to target.
    If used on real robot, this will find the robot namespace.
    If in simulator and only one robot is active, it will be selected.
    However if multiple robots are simulated, then this will not work,
    instead setRobotId must be called explicitly.
    """
    robotId = os.getenv("TURTLE5K_ROBOTNUMBER")
    if robotId != None:
        setRobotId(int(robotId))
    # TODO
    #else:
    #    setRobotId(robotRosInterface.guessRobotId())

def setRobotId(robotId):
    """
    Bind this instance to a robot.
    """
    assert(robotId in range(1, 7))
    global _robotId
    _robotId = robotId
    
def prompt():
    """
    Provide a prompt to users, so they can enter commands.
    """
    print "(enter h for help)"
    while True:
        try:
            s = raw_input("enter command: ")
        except:
            print "error while reading input"
            break
        if s in ["q", "quit", "shutdown"]:
            break
        try:
            parse(s)
        except:
            print "WARNING: something went wrong while evaluating '%s'" % (s)
            traceback.print_exc(file=sys.stdout)

def printHelp():
    print ""
    print "most commands are blocking, which means you cannot type anything until it finished"
    print "use ctrl-C to interrupt such blocking commands and return back to the prompt"
    print ""
    print "COMMANDS:"
    print "  h / help / ?           : show this help"
    print "  q / quit / shutdown    : exit program"
    print "  ctrl-C                 : interrupt running command"
    print "  sleep s                : sleep for s seconds [blocking]"
    print "  stop                   : stop whatever robot is doing"
    print "  bh on|off              : enable/disable ball handlers"
    print "  speed vx vy vphi s     : give a relative speed setpoint (vx,vy,vphi) for s seconds"
    print "  move x y phi           : move robot to (x,y,phi)"
    print "  kick power height      : kick ball with given power and optional height"
    print "  action name [args]     : perform an action [not blocking]"
    print "  actionBlocking name [args] : perform an action [blocking]"
    print "  behavior name          : perform a behavior"
    print "  play                   : change behavior to neutral_playing"
    print "  listScenarios          : shows a list of loaded scenarios"
    print "  scenario name          : run a pre-defined scenario [blocking]"
    print ""
    
def parse(commandString):
    """
    Parse given command string.
    """
    log("parsing command: '%s'" % (commandString), 0)
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
            stop()
        elif mode in ['sleep']:
            sleep(float(args[0]))
        elif mode in ['bh']:
            if len(args) == 1:
                if str(args[0]) == "on":
                    enableBallHandlers()
                if str(args[0]) == "off":
                    disableBallHandlers()
        elif mode in ['speed']:
            vx = float(args[0])
            vy = float(args[1])
            vphi = float(args[2])
            t = float(args[3])
            stop() # stop any previously started action/move
            speed(vx, vy, vphi, t)
        elif mode in ['move', 'target']:
            x = float(args[0])
            y = float(args[1])
            # phi not required, default to zero
            if len(args) > 2:
                phi = float(args[2])
            else:
                phi = 0
            move(x, y, phi)
        elif mode in ['kick']:
            power = float(args[0])
            height = 0
            if len(args) > 1:
                height = float(args[1])
            kick(power, height)
        elif mode in ['action']:
            # an action can have zero or more parameters - just pass it all
            action(*args)
        elif mode in ['behavior']:
            behavior(args[0])
        elif mode in ['play']:
            behavior("neutralPlaying")
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
                        restoreAllConfigs()
            else:
                # relay to scenario selector/executor
                runScenario(*args)
                # After running a scenario, restore all dynamic_reconfigure configs
                # We do this to reset the state of the robot after every scenario
                restoreAllConfigs()
        elif mode in ['analyzeball']:
            monitorBall()
        elif mode in ['analyzeloc']:
            monitorLocalization()
        else:
            log("invalid mode '%s'" % (mode), 0)
    except KeyboardInterruptException:
        # probably a blocking call was interrupted, ok to continue (if prompted)
        log("--interrupted--", 0)
        return
    except:
        log("WARNING: something went wrong while evaluating '%s'" % (commandString), 0)
        traceback.print_exc(file=sys.stdout)
    log("end of parse")
        


