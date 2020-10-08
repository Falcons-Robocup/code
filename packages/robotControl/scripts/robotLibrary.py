""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3
#
# Python library for on-robot commands.



import sys, os, signal
import traceback
import logging
import readline
from time import sleep
import threading
try:
    import EnvironmentField
except:
    print("warning: EnvironmentField is not yet compatible with python3, joystick functionality won't work")

import robotControlInterface


# globals
logging.basicConfig(level=logging.INFO)

def printHelp():
    print(helpText())

def helpText():
    return """
most commands are blocking, which means you cannot type anything until it finished
(TODO: implement ctrl-C to interrupt such blocking commands and return back to the prompt)
COMMANDS:
    h / help / ?           : show this help
    q / quit / shutdown    : exit program
    ctrl-C                 : (TODO) interrupt running command
    stop                   : stop whatever robot is doing
    bh on|off              : enable/disable ball handlers
    velocity vx vy vphi s  : give a relative velocity setpoint (vx,vy,vphi) for s seconds
    move x y phi           : move robot to (x,y,phi) using motionPlanning
    getBall                : get the ball
    kick power height      : (TODO) kick ball with given power (30..180) and optional height (0..180)
    pass x y               : pass to (x,y)
    shoot x y [z]          : straight shot at (x,y,z)
    lob x y [z]            : lob shot at (x,y,z)
    keeper                 : start acting as keeper
    behavior name          : perform a behavior
"""


class RobotLibrary():

    def __init__(self, robotId, joystick=True):
        """
        Setup the robot interface.
        """
        self._robotId = robotId
        self._rci = robotControlInterface.RobotControlInterface(robotId)
        self._rci.connect()
        self._shutdown = False

        # Initialize joystickThread
        # daemon=True means thread is killed during shutdown of main thread, and not block main thread from shutdown
        if joystick:
            self._joystickThreadIsAlive = True
            self._joystickThread = threading.Thread(target=self._joystickThreadExecutor)
            self._joystickThread.setDaemon(True)
            self._joystickThread.start()
        else:
            self._joystickThreadIsAlive = False
            self._joystickThread = None

        self.initializeLogger()

    def initializeLogger(self):
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

    def shutdown(self):
        """
        Cleanup such that robot is in a good state to play (refbox).
        """

        self._joystickThreadIsAlive = False

        # make all threads stop, see e.g. ballMonitor
        self._shutdown = True
        sleep(0.1)
        self._rci.disconnect()

    def prompt(self):
        """
        Provide a prompt to users, so they can enter commands.
        """
        logging.info("(enter h for help)")
        while True:
            try:
                # TODO: prepend with timestamp, nicely align with logging?
                s = input("enter command: ")
            except:
                logging.error("error while reading input")
                break
            if s in ["q", "quit", "shutdown"]:
                break
            try:
                self.parse(s)
            except:
                logging.warning("something went wrong while evaluating '%s'" % (s))
                traceback.print_exc(file=sys.stdout)
        logging.info("shutting down ...")
        self.shutdown()

    def move(self, x, y, phi, slow=False, usePPtol=False, xyTol=0.05, rzTol=0.02):
        bh = self._rci.getBallHandlersEnabled() # maintain current state
        self._rci.setMotionPlanningAction("MOVE", x, y, phi, slow, bh)
        # pathPlanning tolerances are typically configured a bit too tight for smooth testing
        # robot may take a highly-varying amount of time until converged (can be quick, or never...)
        # -> use more relaxed tolerances here, unless explicitly requested ('usePPtol')
        if usePPtol:
            self._rci.blockUntilTPOverridePassedOrFailed()
        else:
            # block, continuously evaluate delta position with given tolerances
            # use delta position as reported in pp diagnostics
            # (alternative: temporarily re-configure pathPlanning?)
            # (alternative: import WorldState library?)
            done = False
            while not done:
                sleep(1.0 / 30)
                if self._shutdown:
                    break
                done = False
                ppDetails = self._rci._robotControl._rtdbGet("DIAG_PATHPLANNING")
                if ppDetails != None:
                    deltaPosition = ppDetails.value['distanceToSubTargetRCS']
                    deltaX = deltaPosition[0]
                    deltaY = deltaPosition[1]
                    deltaRz = deltaPosition[2]
                    done = abs(deltaX) < xyTol and abs(deltaY) < xyTol and abs(deltaRz) < rzTol
        self._rci.clearStimulation()
        # prevent possible last nonzero setpoint causing robot to drift (watchdog)
        self._rci.setRobotVelocity(0.0, 0.0, 0.0)

    def getBall(self):
        self._rci.setTeamplayAction("GET_BALL", {})
        self._rci.blockUntilTPOverridePassedOrFailed()
        self._rci.clearStimulation()

    def interceptBall(self):
        self._rci.setTeamplayAction("INTERCEPT_BALL", {})
        self._rci.blockUntilTPOverridePassedOrFailed()
        self._rci.clearStimulation()

    def passTo(self, x, y):
        self._rci.setMotionPlanningAction("PASS", x, y, 0.0, False, True)
        self._rci.blockUntilTPOverridePassedOrFailed()
        self._rci.clearStimulation()

    def shootAt(self, x, y, z):
        self._rci.setMotionPlanningAction("SHOOT", x, y, z, False, True)
        self._rci.blockUntilTPOverridePassedOrFailed()
        self._rci.clearStimulation()

    def lobShotAt(self, x, y, z):
        self._rci.setMotionPlanningAction("LOB", x, y, z, False, True)
        self._rci.blockUntilTPOverridePassedOrFailed()
        self._rci.clearStimulation()

    def kick(self, power, height=0.0):
        self._rci.setMotionPlanningAction("KICK", power, height, 0.0, False, True)
        self._rci.blockUntilTPOverridePassedOrFailed()
        self._rci.clearStimulation()

    def parse(self, commandString):
        """
        Parse given command string.
        """
        logging.info("parsing command: '%s'" % (commandString))
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
                self._rci.setMotionPlanningAction("STOP", 0.0, 0.0, 0.0, False, False)
            elif mode in ['bh']:
                if len(args) == 1:
                    if str(args[0]) == "on":
                        self._rci.setBallHandlers(True)
                    if str(args[0]) == "off":
                        self._rci.setBallHandlers(False)
            elif mode in ['velocity']:
                vx = float(args[0])
                vy = float(args[1])
                vphi = float(args[2])
                t = float(args[3])
                self._rci.setRobotVelocity(vx, vy, vphi)
                sleep(t)
                self._rci.setRobotVelocity(0.0, 0.0, 0.0)
            elif mode in ['move', 'target']:
                x = float(args[0])
                y = float(args[1])
                # phi not required, default to zero
                if len(args) > 2:
                    phi = float(args[2])
                else:
                    phi = 0.0
                self.move(x, y, phi)
            elif mode in ['getball']:
                if self._robotId == 1:
                    logging.info("skipping: r1 can not grab ball")
                else:
                    self.getBall()
            elif mode in ['pass']:
                x = float(args[0])
                y = float(args[1])
                self.passTo(x, y)
            elif mode in ['shoot']:
                x = float(args[0])
                y = float(args[1])
                # z not required, default to zero
                if len(args) > 2:
                    z = float(args[2])
                else:
                    z = 0
                self.shootAt(x, y, z)
            elif mode in ['lob']:
                x = float(args[0])
                y = float(args[1])
                # z not required, default to zero
                if len(args) > 2:
                    z = float(args[2])
                else:
                    z = 0
                self.lobShotAt(x, y, z)
            elif mode in ['kick']:
                power = float(args[0])
                height = 0
                if len(args) > 1:
                    height = float(args[1])
                self.kick(power, height)
            elif mode in ['keeper']:
                self._rci.setTeamplayBehavior("B_GOALKEEPER", {})
            elif mode in ['behavior']:
                # allow parameters as key=value strings, for instance 'role=defenderMain'
                params = {}
                if len(args) > 1:
                    for s in args[1:]:
                        kv = s.split("=")
                        if len(kv) == 2:
                            params[kv[0]] = kv[1]
                self._rci.setTeamplayBehavior(args[0], params)
                self._rci.blockUntilTPOverridePassedOrFailed()
                self._rci.clearStimulation()
            elif mode in ['listscenarios']:
                print("Loaded scenarios are:")
                print(listScenarios())
                print("\n To execute, type scenario <name>")
            elif mode in ['scenario']:
                # If no arguments given, prompt with a selection menu
                if len(args) == 0:
                    # No argument given, prompt for the scenarios
                    scenarios = listScenarios()
                    scenarios.sort()
                    idx = 1
                    print("Scenarios available for execution:")
                    print("0: Cancel")
                    for scenario in scenarios:
                        print("%s: %s" % (idx, scenario))
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
            """TODO
            except KeyboardInterruptException:
                # probably a blocking call was interrupted, ok to continue (if prompted)
                logging.warning("--interrupted--")
                return
            """
        except:
            logging.warning("something went wrong while evaluating '%s'" % (commandString))
            traceback.print_exc(file=sys.stdout)


    ###################
    # Joystick Thread #
    ###################

    # This thread is always running
    def _joystickThreadExecutor(self):

        while self._joystickThreadIsAlive:
            item = self._rci._robotControl._rtdb2Store.get(0, "JOYSTICK_CONTROL_" + str(self._robotId), timeout=False) # robotId = 0 = coach
            if item != None and item.age() < 1.0:
                self._rci.setBallHandlers(item.value[2])
                self._rci.setRobotVelocity(item.value[1][0], item.value[1][1], item.value[1][2])

                if item.value[4] > 0:
                    self._rci.setMotionPlanningAction("KICK", item.value[4], item.value[3], 0.0, False, True)
                    self._rci.blockUntilTPOverridePassedOrFailed()
                    self._rci.clearStimulation()
                action = item.value[5] # blocking actions
                if action == "getBall":
                    self._rci.setTeamplayAction("GET_BALL", {})
                    self._rci.blockUntilTPOverridePassedOrFailed()
                    self._rci.clearStimulation()
                elif action == "passToTeamMember":
                    # TODO
                    #passToTeamMember()
                    pass
                elif action == "shootAtGoal":
                    goalPos = EnvironmentField.cEnvironmentField.getInstance().getFieldPOI( EnvironmentField.P_OPP_GOALLINE_CENTER )
                    self._rci.setMotionPlanningAction("SHOOT", goalPos.x, goalPos.y, 0.7, False, True)
                    self._rci.blockUntilTPOverridePassedOrFailed()
                    self._rci.clearStimulation()

            elif item != None and item.age() >= 1.0:
                # When the age > 1 second, clearStimulation and remove JOYSTICK_CONTROL such that the next tick the joystick code does not triggered (item == None)
                self._rci.clearStimulation()
                self._rci._robotControl._rtdb2Store.get_and_clear(0, "JOYSTICK_CONTROL_" + str(self._robotId)) # robotId = 0 = coach

            # Sleep to maintain 30Hz
            sleep(1.0 / 30.0)

