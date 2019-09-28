""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import time
import datetime
from Tkinter import *
import ttk
import threading
import math

import roslib
import rospy
import rosgraph
roslib.load_manifest('rosMsgs')
roslib.load_manifest('worldModel')
roslib.load_manifest('teamplay')
import dynamic_reconfigure.client

from worldModel.srv import *
from teamplay.srv import s_tp_input_command, s_tp_input_commandRequest
from rosMsgs.msg import t_robotspeed, t_target

root = Tk()
robotNr = StringVar()
robotNr.set("4") # Default val

calPos = StringVar()
calXY = StringVar()
calStep = StringVar()

expVel = StringVar()
expTime = StringVar()
expDist = StringVar()
expRot = StringVar()
actVel = StringVar()
actTime = StringVar()
actDist = StringVar()
actRot = StringVar()
expVel.set("1")
expTime.set("18")
expDist.set("18")
expRot.set("0.0")

M1TxInit = StringVar()
M1TyInit = StringVar()
M1TwInit = StringVar()
M2TxInit = StringVar()
M2TyInit = StringVar()
M2TwInit = StringVar()
M3TxInit = StringVar()
M3TyInit = StringVar()
M3TwInit = StringVar()
M1TxInit.set("-0.5000")
M1TyInit.set("-0.8660")
M1TwInit.set("0.2200")
M2TxInit.set("-0.5000")
M2TyInit.set("0.8660")
M2TwInit.set("0.2200")
M3TxInit.set("1.0000")
M3TyInit.set("0.0000")
M3TwInit.set("0.2266")

M1TxCal = StringVar()
M1TyCal = StringVar()
M1TwCal = StringVar()
M2TxCal = StringVar()
M2TyCal = StringVar()
M2TwCal = StringVar()
M3TxCal = StringVar()
M3TyCal = StringVar()
M3TwCal = StringVar()

scrollbar = Scrollbar(root)
logbox = Listbox(root, yscrollcommand=scrollbar.set)
scrollbar.config(command=logbox.yview)

setpointPublishing = False


# Before calibration, see if we can disable teamplay (s_tp_input_command, disable?)
# Then when doing the calibration run, set pathPlanning to moveBySpeed, and publish a target of 1 m/s in X or Y.
# After calibration, make sure moveWhileTurning is restored in pathPlanning, and teamplay is reset.
#
# Add a list box to log all output
#
# Make sure there is a stop button that at all times stops the robot.
# Add a "position for calibration" button that places the robot on the position to start driving for calibration, e.g., in the corner of the field.
#
# Add a "reset calibration" button to set the initial values to the matrix:
# [ -0.5000, -0.5000,  1.0000 ] <- 1m/s in X
# [ -0.8660,  0.8660,  0.0000 ] <- 1m/s in Y
# [  0.2200,  0.2200,  0.2266 ] <- 1rad/s in Theta
#      ^        ^        ^
#     Left    Right     Rear   motor
#
# When the Start button is pushed:
# - get_own_location -> x, y, theta
# - start timer
# When the Stop button is pushed, or when the robot manages to drive the full 18 seconds without hitting Stop:
# - get_own_location -> x, y, theta
# - stop timer
# Compute the following:
# - Delta distance -> s(x) for distance in X, s(y) for distance in Y.
# - Delta time (t)
# - Delta orientation (w) (in radians)
#
# We will perform the following three steps:
# 1. Remove rotational error
# 2. Remove drift in perpendicular direction
# 3. Normalize velocity
# 
# If driven in Y, compute Wy:
# Wy = w / t radians
# 
# Then for every motor, compute the calibrated value Ty_cal to update in the matrix.
# Example: Ty for left motor initial value is -0.8660 (see matrix above)
#          Ttheta for left motor initial value is 0.2200 (see matrix above)
# Ty_cal = Ty - Wy * Ttheta
#
# If after x iterations Wy is good enough, it is possible no rotation happens, but the robot drifts over X.
# So, after removing the rotation error, fix the X error.
# Vxy is the drift in X as a result of driving in Y direction:
# Vxy = s(x) / t
#
# Update calibrated value in matrix.
# Example: Left motor Tx = -0.5000
# Ty_cal = Ty - Vxy * Tx
#
# At this point we still need to normalize the velocity.
# Drive on the line for 18s
# s(y)_expected = 18m
# f_y = s(y)_expected / s(y)_actual
# Multiply for all motors Ty with f_y.
#
# At this point, the robot should drive in a straight line at exactly 1m/s.

def log(msg):
    logmsg = time.strftime("%Y/%m/%d %H:%M:%S") + ": " + msg
    logbox.insert(END, logmsg)
    logbox.yview(END)

def DisableAllButtons():
    global btnPosCal
    global btnStopPosCal
    global btnStart
    global btnStop 
    global btnComputeCorr
    global btnApplyCorr
    btnPosCal['state'] = "disabled"
    btnStopPosCal['state'] = "disabled"
    btnStart['state'] = "disabled"
    btnStop['state'] = "disabled" 
    btnComputeCorr['state'] = "disabled"
    btnApplyCorr['state'] = "disabled"

def PPcallback(config):
    returnVal = ""
    try:
        returnVal = "PathPlanning: Algorithm_Type set to {Algorithm_Type} (0=moveWhileTurning, 3=moveAtSpeed)".format(**config)
    except:
        log("Failed to format the returnVal")
    log(returnVal)

def PIcallback(config):
    returnVal = ""
    print config
    M1TxCal.set(config["X1"])
    M2TxCal.set(config["X2"])
    M3TxCal.set(config["X3"])
    M1TyCal.set(config["Y1"])
    M2TyCal.set(config["Y2"])
    M3TyCal.set(config["Y3"])
    M1TwCal.set(config["Theta1"])
    M2TwCal.set(config["Theta2"])
    M3TwCal.set(config["Theta3"])
    try:
        returnVal = "PeripheralsInterface: dynamic_reconfigure callback received"
    except:
        log("Failed to format the returnVal")
    log(returnVal)

class Robot():
    def __init__(self):
        try:
            self.prefix = "/teamA/robot" + robotNr.get() + "/"

            # Check if ROS master is running.
            # The next command (init_node) has no timeout, so there is no graceful error handling possible.
            # Therefore check if /rostopic exists.
            # Throws exception when not found, which is caught.
            rosgraph.Master('/rostopic').getPid()

            # Init ROS node
            rospy.init_node("autocalibration_client")

            # Init PathPlanning dynamic_reconfigure
            self.ppclient = dynamic_reconfigure.client.Client(self.prefix + "PathPlanningNode", timeout=2, config_callback=PPcallback)

            # Init PeripheralsInterface dynamic_reconfigure
            self.piclient = dynamic_reconfigure.client.Client(self.prefix + "peripheralsInterface/motors", timeout=2, config_callback=PIcallback)

            # Init Teamplay override service
            rospy.wait_for_service(self.prefix + "s_tp_input_command", timeout=2)
            self.tpservice = rospy.ServiceProxy(self.prefix + "s_tp_input_command", s_tp_input_command, True)

            # Init WorldModel get_own_location service
            rospy.wait_for_service(self.prefix + "s_get_own_location", timeout=2)
            self.get_own_location_srv = rospy.ServiceProxy(self.prefix + "s_get_own_location", get_own_location, True)

            # Init topic to publish velocity setpoint
            self.publish_velocity_topic = rospy.Publisher(self.prefix + "g_robotspeed", t_robotspeed, queue_size=10)

            # Init topic to publish position setpoint
            self.publish_position_topic = rospy.Publisher(self.prefix + "g_target", t_target, queue_size=10)

            log( "Robot %s initialized." % robotNr.get() )
        except:
            log("Failed to initialize. Is the software of Robot %s running?" % robotNr.get())
        pass

    def get_own_location(self):
        #own_loc_end.ownRobot.Pos.y
        avgX = 0.0
        avgY = 0.0
        avgTheta = 0.0
        nr = 5
        for i in range(nr):
            time.sleep(1)
            loc = self.get_own_location_srv()
            avgX += loc.ownRobot.Pos.x
            avgY += loc.ownRobot.Pos.y
            avgTheta += loc.ownRobot.Pos.theta

        avgX = avgX / nr
        avgY = avgY / nr
        avgTheta = avgTheta / nr
        loc = self.get_own_location_srv()
        loc.ownRobot.Pos.x = avgX
        loc.ownRobot.Pos.y = avgY
        loc.ownRobot.Pos.theta = avgTheta
        return loc

    def set_pathplanning_moveAtSpeed(self):
        log("Setting PathPlanning Algorithm_Type to 3 (moveAtSpeed)")
        try:
            self.ppclient.update_configuration({"Algorithm_Type":3}) #moveAtSpeed
        except rospy.ServiceException, e:
            log("Failed to set PathPlanning Algorithm_Type to 3 (moveAtSpeed)")

    def set_pathplanning_moveWhileTurning(self):
        log("Setting PathPlanning Algorithm_Type to 0 (moveWhileTurning)")
        try:
            self.ppclient.update_configuration({"Algorithm_Type":0}) #moveWhileTurning
        except rospy.ServiceException, e:
            log("Failed to set PathPlanning Algorithm_Type to 0 (moveWhileTurning)")

    def disable_teamplay(self):
        msg = s_tp_input_commandRequest()
        msg.command = "disabled"
        response = self.tpservice(msg)
        return response.result

    def reset_teamplay(self):
        msg = s_tp_input_commandRequest()
        msg.command = "reset"
        response = self.tpservice(msg)
        return response.result

    def publish_target(self, stop=False):

        targetMsg = t_target()

        if (calXY.get() == "Calibrate Y"):
            targetMsg.x = 0.0
            targetMsg.y = int(expVel.get())
            targetMsg.phi = 0.0

        elif (calXY.get() == "Calibrate X"):
            targetMsg.x = int(expVel.get())
            targetMsg.y = 0.0
            targetMsg.phi = 0.0

        #if stop:
        #    targetMsg.vx = 0.0
        #    targetMsg.vy = 0.0
        #    targetMsg.vphi = 0.0

        self.publish_position_topic.publish(targetMsg)

        ### The commented code below uses g_robotspeed (publishing directly to peripheralsInterface) instead of g_target (publishing to pathPlanning)
        ### Since we will be driving using pathPlanning, I think it's best to calibrate using pathPlanning.
        #setpointMsg = t_robotspeed()

        #if (calXY.get() == "Calibrate Y"):
        #    setpointMsg.vx = 0.0
        #    setpointMsg.vy = int(expVel.get())
        #    setpointMsg.vphi = 0.0

        #elif (calXY.get() == "Calibrate X"):
        #    setpointMsg.vx = int(expVel.get())
        #    setpointMsg.vy = 0.0
        #    setpointMsg.vphi = 0.0

        ##if stop:
        ##    setpointMsg.vx = 0.0
        ##    setpointMsg.vy = 0.0
        ##    setpointMsg.vphi = 0.0

        #self.publish_velocity_topic.publish(setpointMsg)

    def position_for_calibration(self):

        targetMsg = t_target()

        if calPos.get() == "Own left corner":
            targetMsg.x = -6.0
            targetMsg.y = -9.0
            if calXY.get() == "Calibrate Y":
                targetMsg.phi = math.pi / 2
            elif calXY.get() == "Calibrate X":
                targetMsg.phi = 0.0
        elif calPos.get() == "Own right corner":
            targetMsg.x = 6.0
            targetMsg.y = -9.0
            if calXY.get() == "Calibrate Y":
                targetMsg.phi = math.pi / 2
            elif calXY.get() == "Calibrate X":
                targetMsg.phi = 0.0
        elif calPos.get() == "Opponent left corner":
            targetMsg.x = -6.0
            targetMsg.y = 9.0
            if calXY.get() == "Calibrate Y":
                targetMsg.phi = -(math.pi / 2)
            elif calXY.get() == "Calibrate X":
                targetMsg.phi = math.pi
        elif calPos.get() == "Opponent right corner":
            targetMsg.x = 6.0
            targetMsg.y = 9.0
            if calXY.get() == "Calibrate Y":
                targetMsg.phi = -(math.pi / 2)
            elif calXY.get() == "Calibrate X":
                targetMsg.phi = math.pi


        self.publish_position_topic.publish(targetMsg)

    def ApplyYCorrection(self, Tleft_cal, Tright_cal, Trear_cal):
        try:
            self.piclient.update_configuration({"Y1":Tleft_cal, "Y2":Tright_cal, "Y3":Trear_cal})
        except rospy.ServiceException, e:
            log("Failed to update matrix")

def InitRobotConnection():
    global robot
    global btnPosCal
    log("Initializing robot connection...")
    robot = Robot()

    # Enable buttons that can now be used
    btnPosCal['state'] = "normal"

def PositionForCalibration():
    global robot
    log("Positioning to %s" % calPos.get())

    # Make sure teamplay is disabled, and pathplanning is on moveWhileTurning.
    robot.disable_teamplay()
    robot.set_pathplanning_moveWhileTurning()

    global setpointPublishing
    setpointPublishing = True
    t = threading.Thread(target=publishCalibrationTarget)
    t.start()

    DisableAllButtons()
    global btnStopPosCal
    btnStopPosCal['state'] = "normal"

def StopPositionForCalibration():
    global robot

    global setpointPublishing
    setpointPublishing = False

    DisableAllButtons()
    global btnPosCal
    btnPosCal['state'] = "normal"
    global btnStart
    btnStart['state'] = "normal"

def StartDriving():
    global robot
    global own_loc_start

    # Disable Teamplay
    robot.disable_teamplay()

    # Set pathplanning to "moveAtSpeed"
    robot.set_pathplanning_moveAtSpeed()

    # Get own_location (or average)
    own_loc_start = robot.get_own_location()

    # Start timer to trigger on expectedTime 
    global setpointPublishing
    setpointPublishing = True
    t = threading.Thread(target=publishSetpoint)
    t.start()

    DisableAllButtons()
    global btnStop
    btnStop['state'] = "normal"

def StopRobot():
    global robot
    global own_loc_start
    global own_loc_end

    # Stop the robot -> Stop publishing target
    global setpointPublishing
    setpointPublishing = False
    #robot.publish_target(stop=True)

    # Reset pathplanning to "moveWhileTurning"
    #robot.set_pathplanning_moveWhileTurning()

    # Get own_location (or average)
    own_loc_end = robot.get_own_location()

    # Compute delta distance. This is always in Y direction because of Field Coordinate System.
    actDist.set( str( own_loc_end.ownRobot.Pos.y - own_loc_start.ownRobot.Pos.y ) )

    # From above, compute velocity
    t = datetime.datetime.strptime(actTime.get(), "%H:%M:%S.%f")
    vel = float(actDist.get()) / t.second
    actVel.set( str(vel) )

    # Finally, compute delta orientation
    actRot.set( str( own_loc_end.ownRobot.Pos.theta - own_loc_start.ownRobot.Pos.theta ) )

    DisableAllButtons()
    global btnComputeCorr
    btnComputeCorr['state'] = "normal"
    global btnStart
    btnStart['state'] = "normal"
    global btnPosCal
    btnPosCal['state'] = "normal"

def ResetMatrix():
    global robot
    robot.ApplyYCorrection(float(M1TyInit.get()), float(M2TyInit.get()), float(M3TyInit.get()))
    log("To be implemented")

def ComputeCorrection():
    # If driven in Y, compute Wy:
    # Wy = w / t radians
    # 
    # Then for every motor, compute the calibrated value Ty_cal to update in the matrix.
    # Example: Ty for left motor initial value is -0.8660 (see matrix above)
    #          Ttheta for left motor initial value is 0.2200 (see matrix above)
    # Ty_cal = Ty - Wy * Ttheta
    w = float(actRot.get()) - float(expRot.get())
    tDat = datetime.datetime.strptime(actTime.get(), "%H:%M:%S.%f")
    t = float(tDat.second)
    wy = w / t

    global Tleft_cal
    global Tright_cal
    global Trear_cal
    #left
    Tleft_cal = float(M1TyCal.get()) - ( wy * float(M1TwCal.get()) )
    log("Left: " + str(Tleft_cal))

    #right
    Tright_cal = float(M2TyCal.get()) - ( wy * float(M2TwCal.get()) )
    log("Right: " + str(Tright_cal))

    #rear
    Trear_cal = float(M3TyCal.get()) - ( wy * float(M3TwCal.get()) )
    log("Rear: " + str(Trear_cal))

    log("ComputeCorrection: To be implemented")
    global btnApplyCorr
    btnApplyCorr['state'] = "normal"

def ApplyCorrection():
    global Tleft_cal
    global Tright_cal
    global Trear_cal
    global robot

    log("Applying matrix update")
    robot.ApplyYCorrection(Tleft_cal, Tright_cal, Trear_cal)

    log("ApplyCorrection: To be implemented")

    global btnResetMatrix
    btnResetMatrix['state'] = "normal"

def ExpectedTimeChanged(*args):
    try:
        int(expTime.get())

        # Compute expected distance with velocity * time
        expDist.set( str( int(expVel.get()) * int(expTime.get()) ) )

    except ValueError:
        log("Error: Expected Time is not an integer.")

def publishSetpoint():
    startTime = datetime.datetime.now()
    global robot
    global setpointPublishing
    while(True):

        # Compute elapsed time
        curTime = datetime.datetime.now()
        elapsed = curTime - startTime
        actTime.set(str(elapsed))

        # If elapsed time > expected time, stop publishing.
        if elapsed > datetime.timedelta(seconds=int(expTime.get())):
            log("Expected time elapsed. Stopping...")
            setpointPublishing = False

        if not setpointPublishing:
            log("Stopping setpoint publishing...")
            break

        # publish expected velocity
        robot.publish_target()

        time.sleep(0.01)

    StopRobot()

def publishCalibrationTarget():
    global robot
    global setpointPublishing

    while(True):

        if not setpointPublishing:
            log("Stopping robot...")
            break

        robot.position_for_calibration()
        time.sleep(0.01)

root.geometry('1500x800')

## Row 0
lblRobotNr = Label(root, text='Robot nr:').grid(row=0, column=0, sticky="e", pady=40)
txtRobotNr = Entry(root, textvariable=robotNr).grid(row=0, column=1)
btnRobotInit = Button(root, text = "Init robot connection", command = InitRobotConnection).grid(row=0, column=2)

## Row 1
cmbCalPos = ttk.Combobox(root, values=["Own left corner", "Own right corner", "Opponent left corner", "Opponent right corner"], state="readonly", textvariable=calPos)
cmbCalPos.current(0)
cmbCalPos.grid(row=1, column=0, pady=40, padx=20)

cmbCalXY = ttk.Combobox(root, values=["Calibrate Y", "Calibrate X"], state="readonly", textvariable=calXY)
cmbCalXY.current(0)
cmbCalXY.grid(row=1, column=1)

btnPosCal = Button(root, text = 'Position for calibration', command = PositionForCalibration, state="disabled")
btnPosCal.grid(row=1, column=2, padx=20)

btnStopPosCal = Button(root, text = 'Stop Robot', command = StopPositionForCalibration, state="disabled")
btnStopPosCal.grid(row=1, column=3)

## Row 2 - 5

lblExpected = Label(root, text="Expected:").grid(row=2, column=0, columnspan=2, sticky="s")
lblExpVel = Label(root, text="Velocity (m/s):").grid(row=3, column=0)
lblExpTime = Label(root, text="Time (s):").grid(row=4, column=0)
lblExpDist = Label(root, text="Distance (m):").grid(row=5, column=0)
lblExpRot = Label(root, text="Orientation (rad):").grid(row=6, column=0)
txtExpVel = Entry(root, textvariable=expVel, state="readonly").grid(row=3, column=1)
txtExpTime = Entry(root, textvariable=expTime).grid(row=4, column=1)
expTime.trace("w", ExpectedTimeChanged)
txtExpDist = Entry(root, textvariable=expDist, state="readonly").grid(row=5, column=1)
txtExpRot = Entry(root, textvariable=expRot, state="readonly").grid(row=6, column=1)

btnStart = Button(root, text = "Start driving", command = StartDriving, state="disabled")
btnStart.grid(row=2, column=2, rowspan=2, sticky="s")
btnStop = Button(root, text = "Stop the robot", command = StopRobot, state="disabled")
btnStop.grid(row=4, column=2, rowspan=2)

lblActual = Label(root, text="Actual:").grid(row=2, column=3, columnspan=2, sticky="s")
lblActVel = Label(root, text="Velocity (m/s):").grid(row=3, column=3)
lblActTime = Label(root, text="Time (s):").grid(row=4, column=3)
lblActDist = Label(root, text="Distance (m):").grid(row=5, column=3)
lblActRot = Label(root, text="Orientation (rad):").grid(row=6, column=3)
txtActVel = Entry(root, textvariable=actVel, state="readonly").grid(row=3, column=4)
txtActTime = Entry(root, textvariable=actTime, state="readonly").grid(row=4, column=4)
txtActDist = Entry(root, textvariable=actDist, state="readonly").grid(row=5, column=4)
txtActRot = Entry(root, textvariable=actRot, state="readonly").grid(row=6, column=4)

cmbCalStep = ttk.Combobox(root, values=["Remove rotational error", "Remove drift in perpendicular direction", "Normalize velocity"], state="readonly", textvariable=calStep)
cmbCalStep.current(0)
cmbCalStep.grid(row=2, column=5, sticky="wes", padx=40, ipadx=40)
btnComputeCorr = Button(root, text = "Compute correction", command=ComputeCorrection, state="disabled")
btnComputeCorr.grid(row=3, rowspan=2, column=5)
btnApplyCorr = Button(root, text = "Apply correction", command=ApplyCorrection, state="disabled")
btnApplyCorr.grid(row=5, rowspan=2, column=5, sticky="n")

lblSpacerExpected = Label(root, text="").grid(row=2, column=6, pady=30)

## Row 7
lblInitialMatrix = Label(root, text="Initial matrix").grid(row=7, column=0, columnspan=3, sticky="s")
lblCurrentMatrix = Label(root, text="Current matrix").grid(row=7, column=3, columnspan=3, sticky="s")
lblSpacerMatrix = Label(root, text="").grid(row=7, column=6, pady=40)

## Row 8-10
txtM1TxInit = Entry(root, textvariable=M1TxInit, state="readonly").grid(row=8, column=0, sticky="e")
txtM1TyInit = Entry(root, textvariable=M1TyInit, state="readonly").grid(row=9, column=0, sticky="e")
txtM1TwInit = Entry(root, textvariable=M1TwInit, state="readonly").grid(row=10, column=0, sticky="e")
txtM2TxInit = Entry(root, textvariable=M2TxInit, state="readonly").grid(row=8, column=1, sticky="we")
txtM2TyInit = Entry(root, textvariable=M2TyInit, state="readonly").grid(row=9, column=1, sticky="we")
txtM2TwInit = Entry(root, textvariable=M2TwInit, state="readonly").grid(row=10, column=1, sticky="we")
txtM3TxInit = Entry(root, textvariable=M3TxInit, state="readonly").grid(row=8, column=2, sticky="w")
txtM3TyInit = Entry(root, textvariable=M3TyInit, state="readonly").grid(row=9, column=2, sticky="w")
txtM3TwInit = Entry(root, textvariable=M3TwInit, state="readonly").grid(row=10, column=2, sticky="w")

txtM1TxCal = Entry(root, textvariable=M1TxCal, state="readonly").grid(row=8, column=3)
txtM1TyCal = Entry(root, textvariable=M1TyCal, state="readonly").grid(row=9, column=3)
txtM1TwCal = Entry(root, textvariable=M1TwCal, state="readonly").grid(row=10, column=3)
txtM2TxCal = Entry(root, textvariable=M2TxCal, state="readonly").grid(row=8, column=4)
txtM2TyCal = Entry(root, textvariable=M2TyCal, state="readonly").grid(row=9, column=4)
txtM2TwCal = Entry(root, textvariable=M2TwCal, state="readonly").grid(row=10, column=4)
txtM3TxCal = Entry(root, textvariable=M3TxCal, state="readonly").grid(row=8, column=5, sticky="w")
txtM3TyCal = Entry(root, textvariable=M3TyCal, state="readonly").grid(row=9, column=5, sticky="w")
txtM3TwCal = Entry(root, textvariable=M3TwCal, state="readonly").grid(row=10, column=5, sticky="w")

## Row 11
btnResetMatrix = Button(root, text = "Reset current matrix to initial values", command=ResetMatrix, state="disabled")
btnResetMatrix.grid(row=11, column=3, columnspan=3)

## Row 12
logbox.grid(row=12, column=0, columnspan=6, sticky="nesw", padx=20, pady=20)
scrollbar.grid(row=12, column=7, sticky="nsw")

root.mainloop()
