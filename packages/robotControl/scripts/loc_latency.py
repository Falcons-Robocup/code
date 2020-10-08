""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3

# Jan Feitsma, February 2020

# Execute loc latency script, which consists of a sequence of setpoints and re-configurations.
# TODO: migrate this to blockly - currently not possible because blockly does not yet support (re-)configuring and worldModel uses old-style ROS configuration
#
# For description and usage hints, execute with '-h'


import sys
import argparse
import falconspy
import rtdb2tools
from robotLibrary import RobotLibrary
from twister import Twister



def parse_arguments():
    parser = argparse.ArgumentParser(description="Execute loc latency scenario. For details, see wiki/calibration/latency-loc.")
    parser.add_argument('-x', help='x position to stand', type=float, default=0.0)
    parser.add_argument('-y', help='y position to stand', type=float, default=0.0)
    parser.add_argument('-v', '--vRz', help='Rz velocity to use', type=float, default=0.5)
    parser.add_argument('--duration', help='duration of single twist in seconds', type=float, default=1.0)
    parser.add_argument('-n', '--numRepeats', help='number of repeats', type=int, default=2)
    return parser.parse_args()


def main(args):
    # move to starting position
    rl = RobotLibrary(args.robot, joystick=False)
    rl.move(args.x, args.y, 0.0)
    # configure
    twister = Twister(args.robot, rci=rl._rci)
    twister.dof = 'Rz' # most stable, due to each wheel contributing the same
    twister.speed = args.vRz
    twister.repeats = args.numRepeats
    twister.duration = args.duration
    twister.disableVisionLoc = True # the point of this procedure is to compare vision with encoders
    twister.accelerationLimit = 2.0 # must prevent wheel slip
    # run
    twister.run()


if __name__ == '__main__':
    args = parse_arguments()

    robot = rtdb2tools.guessAgentId()
    if robot == 0 or robot == None:
        raise RuntimeError("Error: could not determine robot ID, this script should run on a robot")
    args.robot = robot

    main(args)

