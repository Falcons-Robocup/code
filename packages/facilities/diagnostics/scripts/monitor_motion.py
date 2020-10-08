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
import time, datetime
import falconspy
import rtdb2tools



def defaultOutputFile(robot):
    return "/var/tmp/diagnostics_r{:d}_motion_{:s}.txt".format(robot, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))

def parse_arguments():
    descriptionTxt = """Monitor motion performance parameters from RTDB, write continuously to a diagnostics file. Note: where relevant (like robot velocity), data is shown in RCS instead of FCS."""
    exampleTxt = ""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    robot = rtdb2tools.guessAgentId()
    parser.add_argument('-r', '--robot', help='robot ID to use', type=int, default=robot)
    parser.add_argument('-f', '--frequency', help='refresh frequency in Hz', type=float, default=30)
    parser.add_argument('-o', '--output', help='output file', type=str, default=defaultOutputFile(robot))
    parser.add_argument('-s', '--stdout', help='print to stdout instead of to some default file', action='store_true')
    return parser.parse_args()


class LiveMotionMonitor(rtdb2tools.RTDBMonitor):
    def __init__(self, agent, frequency=30.0, output=None):
        if output == None:
            self.output = sys.stdout
        else:
            self.output = open(output, 'w')
        # setup RTDBMonitor
        rtdb2tools.RTDBMonitor.__init__(self, agent=agent, frequency=frequency, rtdbpath=rtdb2tools.RTDB2_DEFAULT_PATH)
        self.prependTimestamp = False
        self.showOnce = False
        self.afterInit = self.printLine
        self.t0 = time.time()
        # setup columns to show
        columns = []
        fmt = "10.4f"
        columns.append(("age", fmt)) # age in seconds since start of monitoring
        columns.append(("ss", "2d")) # ss = shortStroke
        for dof in ["Y", "X", "Rz"]:
            columns.append(("dz" + dof, "2d")) # dz = deadZone
            columns.append(("deltaPos" + dof, fmt))
            columns.append(("currVel" + dof, fmt))
            columns.append(("setVel" + dof, fmt))
            columns.append(("setAcc" + dof, fmt))
            columns.append(("pidProp" + dof, fmt))
            columns.append(("pidInt" + dof, fmt))
            columns.append(("pidDer" + dof, fmt))
        self.columns = columns
        self.data = {}
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
            # note: we could try to calculate actual acceleration, but that data is too noisy
            # -> instead see smoothening and filtering in kstplot_pp using pandas library
        self.subscribe("ROBOT_STATE", handleRobotState)
        def handleDiagPathPlanning(rtdbValue):
            self.data["deltaPosX"] = rtdbValue["distanceToSubTargetRCS"][0]
            self.data["deltaPosY"] = rtdbValue["distanceToSubTargetRCS"][1]
            self.data["deltaPosRz"] = rtdbValue["distanceToSubTargetRCS"][2]
            self.data["setAccX"] = rtdbValue["accelerationRCS"][0]
            self.data["setAccY"] = rtdbValue["accelerationRCS"][1]
            self.data["setAccRz"] = rtdbValue["accelerationRCS"][2]
            self.data["pidPropX"] = rtdbValue["pid"]["x"]["proportional"]
            self.data["pidIntX"] = rtdbValue["pid"]["x"]["integral"]
            self.data["pidDerX"] = rtdbValue["pid"]["x"]["derivative"]
            self.data["pidPropY"] = rtdbValue["pid"]["x"]["proportional"]
            self.data["pidIntY"] = rtdbValue["pid"]["y"]["integral"]
            self.data["pidDerY"] = rtdbValue["pid"]["y"]["derivative"]
            self.data["pidPropRz"] = rtdbValue["pid"]["Rz"]["proportional"]
            self.data["pidIntRz"] = rtdbValue["pid"]["Rz"]["integral"]
            self.data["pidDerRz"] = rtdbValue["pid"]["Rz"]["derivative"]
            self.data["dzX"] = rtdbValue["deadzone"][0]
            self.data["dzY"] = rtdbValue["deadzone"][1]
            self.data["dzRz"] = rtdbValue["deadzone"][2]
            self.data["ss"] = rtdbValue["shortStroke"]
        self.subscribe("DIAG_PATHPLANNING", handleDiagPathPlanning)

    def handle(self, key, item):
        """
        Handle a RTDB item.
        """
        self.stale = not self.fresh(key, item)
        result = self.handles[key](item.value)
        return result

    def printLine(self):
        self.data["age"] = time.time() - self.t0
        headerLine = ""
        valueLine = ""
        for col in self.columns:
            value = 0
            if col[0] in self.data:
                value = self.data[col[0]]
            try:
                width = int(col[1][0:2])
            except:
                width = int(col[1][0])
            formattingValue = "%" + col[1]
            formattingName = "%" + str(width) + "s"
            headerLine += " " + (formattingName % col[0])
            valueLine += " " + (formattingValue % value)
        self.output.write('{0}\n'.format(valueLine)),
        self.output.flush()
            


if __name__ == "__main__":
    # parse arguments
    args = parse_arguments()
    if args.robot == 0:
        raise Exception('could not guess robot id, please specify it')
    # run
    output = args.output
    if args.stdout:
        output = None
    m = LiveMotionMonitor(args.robot, args.frequency, output)
    m.run()

