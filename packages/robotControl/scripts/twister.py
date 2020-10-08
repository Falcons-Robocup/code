""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3

# Jan Feitsma, February 2020

# Basic "twister" motion execution. Used for several verification- and calibration procedures.


import sys, os
import argparse
import time
import falconspy
import rtdb2tools
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH
import robotControlInterface



# constants
DEFAULT_VISION_WEIGHT_FACTOR = 0.05
HEARTBEAT_FREQUENCY = 30.0
# both of these should not be copied/hardcoded here - TODO
# (but both values are stable for years already, so there is not yet much value gained by doing so)


# helper function
def setConfig(robot, component, key, value):
    # TODO implement generically somewhere in a common location (package configuration?)
    # but then all SW clients need to be RTDB-config compatible..
    # for now, only implement what is needed right here
    assert(component == "worldModel")
    # use ROS reconfigure
    cat, subkey = key.split(".")
    command = "rosrun dynamic_reconfigure dynparam set /teamA/robot{}/{}Node/{} {} {}".format(robot, component, cat, subkey, str(value))
    os.system(command)


class Twister():
    """
    Class to perform a twister move. This is used in several verification- and calibration procedures.

    Usage:
    * setup an object for given robot id
    * optional overrule parameters (see below)
    * call run()

    Basic options:
    * dof               : in which direction (degree of freedom) to move, can be 'x', 'y', or 'Rz', default 'Rz'
    * speed             : speed of twist movement (m/s or rad/s, depending on 'dof')
    * duration          : duration of twist movement
    * sleep             : time to sleep in between twist movements
    * repeats           : number of forth-and-back twists to execute

    Advanced options:
    * disableVisionLoc  : disable the influence of vision on worldModel position, i.e. use purely encoders
    * accelerationLimit : if set, apply limit during twist to prevent wheel slip
    """
    def __init__(self, robot, rci=None):
        self.robot = int(robot)
        # settings which can be customized
        self.dof = 'Rz'
        self.speed = 0.5
        self.duration = 1.0
        self.sleep = 1.0
        self.repeats = 2
        self.disableVisionLoc = True
        self.accelerationLimit = None
        # remaining initialization
        self.rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH, False)
        item = self.rtdb2Store.get(robot, "ROBOT_STATE")
        if item == None:
            raise RuntimeError("Error: robot software not active")
        if rci == None:
            rci = robotControlInterface.RobotControlInterface(self.robot)
            rci.connect()
        self.rci = rci

    def run(self):
        # twist execution
        for it in range(self.repeats):
            self.twist()

    def twist(self):
        self._setSpeed(1)
        time.sleep(self.sleep)
        self._setSpeed(-1)
        time.sleep(self.sleep)

    def _setSpeed(self, speedFactor):
        # determine velocity setpoint
        velocity = [0.0, 0.0, 0.0]
        dof2idx = {'x': 0, 'y': 1, 'Rz': 2}
        if not dof2idx.has_key(self.dof):
            raise RuntimeError("Error: invalid setting dof={}, must be one of ({})".format(self.dof, ','.join(dof2idx.keys())))
        velocity[dof2idx[self.dof]] = speedFactor * self.speed
        # disable vision loc (optional)
        if self.disableVisionLoc:
            setConfig(self.robot, "worldModel", "localization.visionOwnWeightFactor", 0.0)
        # move
        if self.accelerationLimit == None:
            # simple abrupt continuous velocity setpoint (robot wheels might start to slip if set too large)
            self.rci.setRobotVelocity(*velocity)
            time.sleep(self.duration)
            self.rci.setRobotVelocity(0.0, 0.0, 0.0) # done, reset setpoint
        else:
            # explicit stimulation, slowly increasing acceleration
            # useful to prevent wheel slip at higher speed setpoints
            # note: we do not bother yet to decelerate
            # (this might be useful though if robot would be actively braking, causing some friction/slip)
            t = 0.0
            dt = 1.0 / HEARTBEAT_FREQUENCY
            v = 0.0
            while t < self.duration:
                v += dt * self.accelerationLimit
                if v > self.speed:
                    v = self.speed
                velocity[dof2idx[self.dof]] = speedFactor * v
                self.rci.setRobotVelocity(*velocity)
                time.sleep(dt)
                t += dt
            self.rci.setRobotVelocity(0.0, 0.0, 0.0) # done, reset setpoint
        # restore vision loc (optional)
        if self.disableVisionLoc:
            setConfig(self.robot, "worldModel", "localization.visionOwnWeightFactor", DEFAULT_VISION_WEIGHT_FACTOR)
            # note: worldModel typically needs a bit of time to settle, see sleeping in twist()


def main(args):
    twister = Twister(args.robot)
    # configure
    twister.dof = args.dof
    twister.speed = args.speed
    twister.duration = args.duration
    twister.sleep = args.sleep
    twister.repeats = args.numRepeats
    twister.disableVisionLoc = not args.keepVision
    twister.accelerationLimit = args.acceleration
    # run
    twister.run()


def parse_arguments():
    parser = argparse.ArgumentParser(description="Execute a twist movement sequence.")
    parser.add_argument('--dof', help='direction (degree of freedom) to move', type=str, choices=['x', 'y', 'Rz'], default='Rz')
    parser.add_argument('-v', '--speed', help='speed to use', type=float, default=0.5)
    parser.add_argument('-d', '--duration', help='duration of single twist in seconds', type=float, default=1.0)
    parser.add_argument('-s', '--sleep', help='sleep duration in between twists', type=float, default=1.0)
    parser.add_argument('-n', '--numRepeats', help='number of repeats', type=int, default=2)
    parser.add_argument('-V', '--keepVision', help='do not disable vision loc during twist', action='store_true')
    parser.add_argument('-a', '--acceleration', help='acceleration limit', type=float)
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()

    robot = rtdb2tools.guessAgentId()
    if robot == 0 or robot == None:
        raise RuntimeError("Error: could not determine robot ID, this script should run on a robot")
    args.robot = robot

    main(args)

