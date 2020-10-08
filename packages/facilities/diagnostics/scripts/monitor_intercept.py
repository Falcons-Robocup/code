""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python3


import os, sys
import argparse
import math
import falconspy
import rtdb2tools



def parse_arguments():
    descriptionTxt = """Monitor intercept performance parameters from RTDB, write continuously to stdout. """
    exampleTxt = """Examples:
    monitor_intercept.py
    monitor_intercept.py -r 2

Example output:    TODO (desc)
"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-r', '--robot', help='robot ID to use', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument('-f', '--frequency', help='refresh frequency in Hz', type=float, default=10)
    return parser.parse_args()


class LiveInterceptMonitor(rtdb2tools.RTDBMonitor):
    def __init__(self, agent, frequency=30.0):
        # setup RTDBMonitor
        rtdb2tools.RTDBMonitor.__init__(self, agent=agent, frequency=frequency, rtdbpath=rtdb2tools.RTDB2_DEFAULT_PATH)
        self.prependTimestamp = False
        self.showOnce = False
        self.afterInit = self.printLine
        self.robotID = agent
        # setup columns to show
        self.columns = [("hasBall", "ok", "2d"),
                        ("bhLeft", "bhLeft", "7d"),
                        ("bhRight", "bhRight", "7d"),
                        ("visBallDst", "visBallDst", "10.4f"),
                        ("visBallEl", "visBallEl", "10.4f"),
                        ("visBallAz", "visBallAz", "10.4f"),
                        ("deltaPosX", "deltaPosX", "10.4f"),
                        ("deltaPosRz", "deltaPosRz", "10.4f"),
                        ("shortStroke", "ss", "2d"),
                        ("currVelX", "currVelX", "10.4f"),
                        ("currVelRz", "currVelRz", "10.4f"),
                        ("setVelX", "setVelX", "10.4f"),
                        ("setVelRz", "setVelRz", "10.4f")]
        self.data = {}
        self.data["visBallAz"] = float('nan')
        self.data["visBallEl"] = float('nan')
        self.data["visBallDst"] = float('nan')
        self.stale = False
        self.lastStale = False
        # setup connections and interpreters of useful data
        def handleRobotVelocitySetpoint(rtdbValue):
            self.data["setVelX"] = rtdbValue[0]
            self.data["setVelY"] = rtdbValue[1]
            self.data["setVelRz"] = rtdbValue[2]
        self.subscribe("ROBOT_VELOCITY_SETPOINT", handleRobotVelocitySetpoint)
        # TODO: factor out common parts (see kstplot_pp)
        def transform(velocity, robotRz):
            # copied from FalconsCoordinates.py in packages/geometry
            angle = (math.pi*0.5 - robotRz)
            s = math.sin(angle)
            c = math.cos(angle)
            return (c * velocity[0] - s * velocity[1], s * velocity[0] + c * velocity[1], velocity[2])
        def handleRobotState(rtdbValue):
            vRcs = transform(rtdbValue[3], rtdbValue[2][2])
            self.data["currVelX"] = vRcs[0]
            self.data["currVelY"] = vRcs[1]
            self.data["currVelRz"] = vRcs[2]
            self.data["hasBall"] = int(rtdbValue[4])
            # note: we could try to calculate actual acceleration, but that data is too noisy
            # -> instead see smoothening and filtering in kstplot_pp using pandas library
        self.subscribe("ROBOT_STATE", handleRobotState)
        def handleDiagPathPlanning(rtdbValue):
            try:
                self.data["deltaPosX"] = rtdbValue["distanceToSubTargetRCS"][0]
                self.data["deltaPosY"] = rtdbValue["distanceToSubTargetRCS"][1]
                self.data["deltaPosRz"] = rtdbValue["distanceToSubTargetRCS"][2]
                self.data["shortStroke"] = rtdbValue["shortStroke"]
            except:
                self.data["deltaPosX"] = float('nan')
                self.data["deltaPosY"] = float('nan')
                self.data["deltaPosRz"] = float('nan')
                self.data["shortStroke"] = 0
        self.subscribe("DIAG_PATHPLANNING", handleDiagPathPlanning)
        def handleDiagWorldModelLocal(rtdbValue):
            self.data["visBallAz"] = float('nan')
            self.data["visBallEl"] = float('nan')
            self.data["visBallDst"] = float('nan')
            # require worldModel to track the ball
            if len(rtdbValue["balls"]) > 0:
                # we expect the following:
                # * only the best tracker is logged
                # * measurements are sorted by timestamp, newest at the end
                # find newest measurement of this robot
                for m in rtdbValue["balls"][0]["measurements"]:
                    if m["used"] and m["m"][0][0] == self.robotID:
                        self.data["visBallAz"] = math.pi * 0.5 - m["m"][4] # correct annoying legacy CS offsets
                        self.data["visBallEl"] = m["m"][5]
                        self.data["visBallDst"] = m["m"][6]
        self.subscribe("DIAG_WORLDMODEL_LOCAL", handleDiagWorldModelLocal)
        def handleBallHandlersFeedback(rtdbValue):
            self.data["bhLeft"] = rtdbValue[0]
            self.data["bhRight"] = rtdbValue[1]
        self.subscribe("BALLHANDLERS_FEEDBACK", handleBallHandlersFeedback)
        # TODO hide sensor angles internally, for diagnostics we need sensible values mapping to [0.0 .. 1.0]

    def handle(self, key, item):
        """
        Handle a RTDB item.
        """
        self.stale = not self.fresh(key, item)
        result = self.handles[key](item.value)
        return result

    def printLine(self):
        # note: it is not needed to re-calculate the headerLine each iteration, but also not expensive
        # normally header line is shown on top of a table, but in this continuous setting this is useless...
        # we could display it every say 2 seconds, but still it would not be readible
        # trick: use carriage return to always display column heading as _last_ line
        headerLine = ""
        valueLine = ""
        for col in self.columns:
            value = self.data[col[0]]
            name = col[1]
            try:
                width = int(col[2][0:2])
            except:
                width = int(col[2][0])
            formattingValue = "%" + col[2]
            formattingName = "%" + str(width) + "s"
            headerLine += " " + (formattingName % name)
            valueLine += " " + (formattingValue % value)
        if self.stale:
            if not self.lastStale:
                print("") # only first time, to not make the 'header' line disappear
            # TODO: report how long data was stale?
            valueLine = "stale" + " " * (len(valueLine) - 5) + "\r"
            print('\r{0}'.format(valueLine)),
        else:
            print('\r{0}\n{1}'.format(valueLine, headerLine)),
        sys.stdout.flush()
        self.lastStale = self.stale


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    if args.robot == 0:
        raise Exception('could not guess robot id, please specify it')
    # run
    m = LiveInterceptMonitor(args.robot, args.frequency)
    m.run()

