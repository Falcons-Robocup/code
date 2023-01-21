# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
# Python command-line interface for commands, on-robot.


import os
import sys
import traceback
import argparse
import logging
import robotLibrary


# globals
logging.basicConfig(level=logging.INFO)


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

def parse(commandString, robotLib):
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
            robotLib.stop()
        elif mode in ['bh']:
            if len(args) == 1:
                if str(args[0]) == "on":
                    robotLib.enableBallHandlers()
                if str(args[0]) == "off":
                    robotLib.disableBallHandlers()
        elif mode in ['velocity']:
            vx = float(args[0])
            vy = float(args[1])
            vphi = float(args[2])
            duration = float(args[3])
            robotLib.velocitySetpoint(vx, vy, vphi, duration)
        elif mode in ['move', 'target']:
            x = float(args[0])
            y = float(args[1])
            # phi not required, default to zero
            if len(args) > 2:
                phi = float(args[2])
            else:
                phi = 0.0
            robotLib.move(x, y, phi)
        elif mode in ['getball']:
            if robotLib._robotId == 1:
                logging.info("skipping: r1 can not grab ball")
            else:
                robotLib.getBall()
        elif mode in ['pass']:
            x = float(args[0])
            y = float(args[1])
            robotLib.passTo(x, y)
        elif mode in ['shoot']:
            x = float(args[0])
            y = float(args[1])
            # z not required, default to zero
            if len(args) > 2:
                z = float(args[2])
            else:
                z = 0
            robotLib.shootAt(x, y, z)
        elif mode in ['lob']:
            x = float(args[0])
            y = float(args[1])
            # z not required, default to zero
            if len(args) > 2:
                z = float(args[2])
            else:
                z = 0
            robotLib.lobShotAt(x, y, z)
        elif mode in ['kick']:
            power = float(args[0])
            height = 0
            if len(args) > 1:
                height = float(args[1])
            robotLib.kick(power, height)
        elif mode in ['keeper']:
            robotLib.keeper()
        elif mode in ['behavior']:
            # allow parameters as key=value strings, for instance 'role=defenderMain'
            params = {}
            if len(args) > 1:
                for s in args[1:]:
                    kv = s.split("=")
                    if len(kv) == 2:
                        params[kv[0]] = kv[1]
            robotLib.behavior(args[0], params)
        elif mode in ['scenarios']:
            robotLib.scenarioInfo()
            print("\nTo execute, type scenario <name> optionally followed by arguments")
        elif mode in ['scenario']:
            # relay to scenario selector/executor
            robotLib.scenarioRun(*args)
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

def prompt(robotLib):
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
            parse(s, robotLib)
        except:
            logging.warning("something went wrong while evaluating '%s'" % (s))
            traceback.print_exc(file=sys.stdout)
    logging.info("shutting down ...")

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
    getball                : get the ball
    kick power height      : (TODO) kick ball with given power (30..180) and optional height (0..180)
    pass x y               : pass to (x,y)
    shoot x y [z]          : straight shot at (x,y,z)
    lob x y [z]            : lob shot at (x,y,z)
    keeper                 : start acting as keeper
    behavior name          : perform a behavior
    scenarios              : list available scenarios
    scenario name          : perform a scenario
"""

###################
# Joystick Thread #
###################

def initJoystickThread():
    # Initialize joystickThread
    # daemon=True means thread is killed during shutdown of main thread, and not block main thread from shutdown
    _joystickThreadIsAlive = True
    _joystickThread = threading.Thread(target=_joystickThreadExecutor)
    _joystickThread.setDaemon(True)
    _joystickThread.start()

# This thread is always running
def _joystickThreadExecutor():

    while self._joystickThreadIsAlive:
        item = self._rci._robotControl._rtdb2Store.get(0, "JOYSTICK_CONTROL_" + str(self._robotId), timeout=False) # robotId = 0 = coach
        if item != None and item.age() < 1.0:
            self._rci.setBallHandlers(item.value[2])
            self._rci.setRobotVelocity(item.value[1][0], item.value[1][1], item.value[1][2])

            if item.value[4] > 0:
                self._rci.setMotionPlanningAction("KICK", item.value[4], item.value[3], 0.0, "NORMAL", True)
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
                self._rci.setMotionPlanningAction("SHOOT", goalPos.x, goalPos.y, 0.7, "NORMAL", True)
                self._rci.blockUntilTPOverridePassedOrFailed()
                self._rci.clearStimulation()

        elif item != None and item.age() >= 1.0:
            # When the age > 1 second, clearStimulation and remove JOYSTICK_CONTROL such that the next tick the joystick code does not triggered (item == None)
            self._rci.clearStimulation()
            self._rci._robotControl._rtdb2Store.get_and_clear(0, "JOYSTICK_CONTROL_" + str(self._robotId)) # robotId = 0 = coach

        # Sleep to maintain 30Hz
        time.sleep(1.0 / 30.0)

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

if __name__ == '__main__':

    # argument parsing
    parser     = argparse.ArgumentParser(description='robot command interface', epilog=helpText(), formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robotId', '-r', help='robot number', default=None, type=int)
    parser.add_argument('command', nargs='?', help='command string to execute', default=None)
    args, leftovers = parser.parse_known_args()

    # If no robotId given, obtain the Id from environment variable (real robot only)
    if args.robotId == None:
        robotId = guessRobotId()
    else:
        robotId = args.robotId

    initializeLogger()

    # Instantiate and initialize RobotLibrary
    robotLib = robotLibrary.RobotLibrary(robotId)

    if not robotLib.isInMatchMode():
        print("Error: Robot r%s is already in TestMode." % (robotId))
        print("Please wait for ongoing scenarios to finish, or restart the robot software.")
        sys.exit(1)

    robotLib.connect()

    # Initialize joystick thread
    # initJoystickThread()
    
    # has a command been given? if not, provide a prompt
    if args.command != None:
        commandStr = args.command + " " + ' '.join(leftovers)
        parse(commandStr, robotLib)
    else:
        prompt(robotLib)
    
    robotLib.disconnect()
    sys.exit(0)

