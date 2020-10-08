""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python3

from Tkinter import *
import os
import random
import rospy
import sys
import thread
import time
import tkMessageBox
import Image
import Tkinter
from collections import defaultdict
from newest_logdir import newest_logdir
from sensor_msgs.msg import Joy
from time import localtime, sleep, strftime
from tkFileDialog import *
from tkMessageBox import *
from ttk import Notebook
from udpInterface import udpInterface

IMG_LOCATION         = "/home/robocup/falcons/code/packages/coachCommands/"
GIF_STOP_BUTTON      = IMG_LOCATION + "StopButton.gif"
GIF_RESET_BUTTON     = IMG_LOCATION + "ResetButton.gif"
GIF_LEFT_ARROW       = IMG_LOCATION + "LeftArrow.gif"
GIF_RIGHT_ARROW      = IMG_LOCATION + "RightArrow.gif"
GIF_FALCONS_LOGO     = IMG_LOCATION + "FalconsLogo.gif"

CMD_MATCH            = "Match"
CMD_DEMO             = "Demo"
CMD_TEST             = "Test"
CMD_SETTINGS         = "Settings"

CMD_CLAIM            = "Claim"
CMD_RELEASE          = "Release"
CMD_SELECT           = "Select"
CMD_DESELECT         = "Deselect"
CMD_START            = "Start"
CMD_STOP             = "Stop"
CMD_REBOOT           = "Reboot"

CMD_PARK             = "Park"
CMD_PREPARE          = "Prepare"
CMD_START_GAME       = "Start"
CMD_STOP_GAME        = "Stop"

CMD_ENABLE_CAMERA    = "Enable Camera"
CMD_DISABLE_CAMERA   = "Disable Camera"
CMD_ENABLE_JOYSTICK  = "Enable Joystick"
CMD_DISABLE_JOYSTICK = "Disable Joystick"
CMD_ENABLE_KEYBOARD  = "Enable Keys"
CMD_DISABLE_KEYBOARD = "Disable Keys"

CMD_GO_HOME          = "Go Home"
CMD_MAKE_SQUARE      = "Make Square"
CMD_DANCE            = "Dance"
CMD_GET_BALL         = "Get Ball"
CMD_SHOOT_AT_GOAL    = "Shoot Ball"
CMD_GOALKEEPER       = "Goalie"

INDEX_CLAIM          = 0
INDEX_SELECT         = 1
INDEX_SOFTWARE       = 2
INDEX_REBOOT         = 3

BUTTON_HEIGHT        = 1
BUTTON_WIDTH         = 8
ROBOT_BUTTON_WIDTH   = 4
CONFIG_BUTTON_WIDTH  = 11

PAD_X                = 10
PAD_Y                = 15

ARROW_WIDTH          = 85
ARROW_HEIGHT         = 21

PARKING_WIDTH        = 85
PARKING_HEIGHT       = 21

STOP_WIDTH           = 75
STOP_HEIGHT          = 75

RESET_WIDTH          = 42
RESET_HEIGHT         = 42

LOGGING_WIDTH        = 55
LOGGING_HEIGHT       = 3

COLOR_ACTIVATED      = "pale green"
COLOR_CLAIMED        = "pale green"
COLOR_DESELECTED     = "sandy brown"

SLEEP_5_SECONDS      = "sleep 5"
SLEEP_7_SECONDS      = "sleep 7"
SLEEP_9_SECONDS      = "sleep 9"
SLEEP_10_SECONDS     = "sleep 10"

NR_OF_ROBOTS         = 6

SAMPLE_FREQUENCY     = 10 # Hz

MAX_KICK_HEIGHT      = 10
MAX_LEVER_VALUE      =  1
MIN_LEVER_VALUE      = -1

inject_noise =  True
parkOrientation = 0

def joystickCallback(joy):
    global speedX
    global speedY
    global speedPhi
    global kickHeight
    global kickBall
    global changeSpeed
#    writeLogging('joystick input:\n' + str(joy))
    speedX    = joy.axes[0] * drivingSpeed
    speedY    = joy.axes[1] * drivingSpeed
    speedPhi  = joy.axes[2] * drivingSpeed * 2.0
    changeSpeed = True
    kickHeight = (joy.axes[3] - MIN_LEVER_VALUE) * (MAX_KICK_HEIGHT / (MAX_LEVER_VALUE - MIN_LEVER_VALUE) )
    if joy.buttons[0] == 1 or joy.buttons[6] == 1 or joy.buttons[7] == 1:
        kickBall = True

def processJoystickInputInNewThread(robot):
    global speedX
    global speedY
    global speedPhi
    global kickHeight
    global kickBall
    global changeSpeed
    joystickSubscriber = rospy.Subscriber('/joy', Joy, joystickCallback)
    speedX = 0.0
    speedY = 0.0
    speedPhi = 0.0
    kickHeight = 0.0
    u.sendTo(robot, "action stop")
    while joystickEnabled:
        if changeSpeed or speedX != 0 or speedY != 0 or speedPhi != 0:
            if inject_noise:
                r=random.uniform(0.99, 0.9999)
                cmd = "speed %f %f %f %f" % ( r*speedX, r*speedY, r*speedPhi, r*(1.0 / SAMPLE_FREQUENCY) )
            else:
                cmd = "speed %f %f %f %f" % ( speedX, speedY, speedPhi, (1.0 / SAMPLE_FREQUENCY) )
            writeLogging(cmd)
            u.sendTo(robot, cmd)
            changeSpeed = False
        if kickBall:
            cmd = "kick %f %f" % (kickHeight, kickPower)
            writeLogging(cmd)
            u.sendTo(robot, cmd)
            kickBall = False
        sleep(1.0 / SAMPLE_FREQUENCY)
    joystickSubscriber.unregister()
    u.sendTo(robot, "action stop")

def processKeyboardInputInNewThread(robot):
    global speedX
    global speedY
    global speedPhi
    global kickBall
    global changeSpeedX
    global changeSpeedY
    global changeSpeedPhi
    speedX = 0.0
    speedY = 0.0
    speedPhi = 0.0
    u.sendTo(robot, "action stop")
    while keyboardEnabled:
        if changeSpeedX or changeSpeedY or changeSpeedPhi:
            if inject_noise:
                r=random.uniform(0.99, 0.9999)
                cmd = "speed %f %f %f %f" % ( r*speedX, r*speedY, r*speedPhi, r*(1.0 / SAMPLE_FREQUENCY) )
            else:
                cmd = "speed %f %f %f %f" % ( speedX, speedY, speedPhi, (1.0 / SAMPLE_FREQUENCY) )
            writeLogging(cmd)
            u.sendTo(robot, cmd)
        if kickBall:
            cmd = "kick %f %f" % (kickHeight, kickPower)
            writeLogging(cmd)
            u.sendTo(robot, cmd)
            kickBall = False
        sleep(1.0 / SAMPLE_FREQUENCY)
    u.sendTo(robot, "action stop")

def executeActionSequenceInNewThread(robot, sequence):
    global sequenceRunning
    i = 0
    nrSteps = len(sequence)
    u.sendTo(robot, "action stop")
    while i < nrSteps and sequenceRunning != False:
        writeLogging("Step %d = %s" % (i+1,sequence[i]))
        if sequence[i] == SLEEP_5_SECONDS:
            sleep(5.0)
        elif sequence[i] == SLEEP_7_SECONDS:
            sleep(7.0)
        elif sequence[i] == SLEEP_9_SECONDS:
            sleep(9.0)
        elif sequence[i] == SLEEP_10_SECONDS:
            sleep(10.0)
        else:
            u.sendTo(robot, sequence[i])
        i += 1
    sequenceRunning = False
    if i == nrSteps:
        writeLogging("Sequence finished successfully")
    else:
        writeLogging("Sequence interrupted")
    executeAction("action stop")
    
def executeActionSequence(sequence):
    global sequenceRunning
    robot = getFirstSelectedRobot()
    if robot == None:
        tkMessageBox.showerror("Precondition Error", "No robot selected")
    else:
        sequenceRunning = True
        thread.start_new_thread( executeActionSequenceInNewThread, (robot, sequence,) )

def executeAction(cmd):
    robot = getFirstSelectedRobot()
    if robot == None:
        tkMessageBox.showerror("Precondition Error", "No robot selected")
    else:
        for robot in getAllSelectedRobots():
            writeLogging("Send %s to robot %d" % (cmd, robot))
            u.sendTo(robot, cmd)

def selectMatchMode():
    global demoModeActive
    global testModeActive
    writeLogging("Match Mode selected")
    buttonMainMode1.config(relief = FLAT, background = COLOR_ACTIVATED, state = DISABLED)
    buttonMainMode2.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    buttonMainMode3.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    buttonMainMode4.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    frameMatchMode.pack(side = TOP, fill = X, expand = True)
    frameDemoMode1.pack_forget()
    frameDemoMode2.pack_forget()
    frameDemoMode3.pack_forget()
    frameTestMode1.pack_forget()
    frameTestMode2.pack_forget()
    frameTestMode3.pack_forget()
    frameSettings2.pack_forget()
    if loggingShown:
        frameLogging.pack(side = BOTTOM)
    demoModeActive = False
    testModeActive = False

def selectDemoMode():
    global demoModeActive
    global testModeActive
    writeLogging("Demo Mode selected")
    buttonMainMode2.config(relief = FLAT, background = COLOR_ACTIVATED, state = DISABLED)
    buttonMainMode1.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    buttonMainMode3.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    buttonMainMode4.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    frameDemoMode1.pack(side = TOP, fill = X, expand = True)
    frameDemoMode2.pack(side = TOP, fill = X, expand = True)
    frameDemoMode3.pack(side = TOP, fill = X, expand = True)
    frameMatchMode.pack_forget()
    frameTestMode1.pack_forget()
    frameTestMode2.pack_forget()
    frameTestMode3.pack_forget()
    frameSettings2.pack_forget()
    if loggingShown:
        frameLogging.pack(side = BOTTOM)
    demoModeActive = True
    testModeActive = False
    nrRobotsSelected = getNrRobotsSelected()
    if nrRobotsSelected > 2:
        tkMessageBox.showerror("Precondition Error", "Not allowed to select more than 2 robots in Demo Mode: deselecting all claimed robots now")
        robots = range(1, NR_OF_ROBOTS+1)
        for robot in robots:
            selectRobot(robot, select = False, update = False)
        enableDisableDemoButtons()

def selectTestMode():
    global demoModeActive
    global testModeActive
    writeLogging("Test Mode selected")
    buttonMainMode3.config(relief = FLAT, background = COLOR_ACTIVATED, state = DISABLED)
    buttonMainMode1.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    buttonMainMode2.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    buttonMainMode4.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    frameTestMode1.pack(side = TOP, fill = X, expand = True)
    frameTestMode2.pack(side = TOP, fill = X, expand = True)
    frameTestMode3.pack(side = TOP, fill = X, expand = True)
    frameMatchMode.pack_forget()
    frameDemoMode1.pack_forget()
    frameDemoMode2.pack_forget()
    frameDemoMode3.pack_forget()
    frameSettings2.pack_forget()
    if loggingShown:
        frameLogging.pack(side = BOTTOM)
    demoModeActive = False
    testModeActive = True

def selectSettings():
    global demoModeActive
    global testModeActive
    writeLogging("Settings selected")
    buttonMainMode4.config(relief = FLAT, background = COLOR_ACTIVATED, state = DISABLED)
    buttonMainMode1.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    buttonMainMode2.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    buttonMainMode3.config(relief = RAISED, background = DefaultColor, state = NORMAL)
    frameSettings2.pack(side = TOP, fill = X, expand = True)
    frameMatchMode.pack_forget()
    frameDemoMode1.pack_forget()
    frameDemoMode2.pack_forget()
    frameDemoMode3.pack_forget()
    frameTestMode1.pack_forget()
    frameTestMode2.pack_forget()
    frameTestMode3.pack_forget()
    if loggingShown:
        frameLogging.pack(side = BOTTOM)
    demoModeActive = False
    testModeActive = False

def stopButtonPressed():
    global sequenceRunning
    writeLogging("Stop button pressed")
    if sequenceRunning == True:
        sequenceRunning = False
    else:
        executeAction("action stop")

def resetButtonPressed():
    writeLogging("Reset button pressed")
    robots = range(1, NR_OF_ROBOTS+1)
    for robot in robots:
        releaseRobot( robot )

def showAbout():
    windowAbout.deiconify()

def closewindowAbout():
    windowAbout.withdraw()

def enableDisableDemoButtons():
    nrRobotsSelected = getNrRobotsSelected()
    if nrRobotsSelected == 1:
        buttonDemoMode11.config(state=NORMAL)
        buttonDemoMode12.config(state=NORMAL)
        buttonDemoMode13.config(state=NORMAL)
        buttonDemoMode21.config(state=NORMAL)
        buttonDemoMode22.config(state=NORMAL)
        buttonDemoMode23.config(state=NORMAL)
        buttonDemoMode24.config(state=NORMAL)
        buttonDemoMode25.config(state=NORMAL)
        buttonDemoMode31.config(state=DISABLED)
        buttonDemoMode32.config(state=DISABLED)
    elif nrRobotsSelected == 2:
        buttonDemoMode11.config(state=DISABLED)
        buttonDemoMode12.config(state=DISABLED)
        buttonDemoMode13.config(state=DISABLED)
        buttonDemoMode21.config(state=DISABLED)
        buttonDemoMode22.config(state=DISABLED)
        buttonDemoMode23.config(state=DISABLED)
        buttonDemoMode24.config(state=DISABLED)
        buttonDemoMode25.config(state=DISABLED)
        buttonDemoMode31.config(state=NORMAL)
        buttonDemoMode32.config(state=NORMAL)
    else:
        buttonDemoMode11.config(state=DISABLED)
        buttonDemoMode12.config(state=DISABLED)
        buttonDemoMode13.config(state=DISABLED)
        buttonDemoMode21.config(state=DISABLED)
        buttonDemoMode22.config(state=DISABLED)
        buttonDemoMode23.config(state=DISABLED)
        buttonDemoMode24.config(state=DISABLED)
        buttonDemoMode25.config(state=DISABLED)
        buttonDemoMode31.config(state=DISABLED)
        buttonDemoMode32.config(state=DISABLED)
    if nrRobotsSelected > 2 and demoModeActive == True:
        tkMessageBox.showerror("Precondition Error", "Not allowed to select more than 2 robots in Demo Mode")
    
def robotButtonPressed(robot, cmd):
    if cmd == CMD_CLAIM:
        if robotClaimed[robot]:
            releaseRobot(robot)
        else:
            claimRobot(robot)
    elif cmd == CMD_SELECT:
        if robotSelected[robot]:
            selectRobot(robot, select = False, update = True)
        else:
            selectRobot(robot, select = True, update = True)
    elif cmd == CMD_START:
        toggleRobotSoftware(robot)
    elif cmd == CMD_REBOOT:
        rebootRobot(robot)

def claimRobot(robot):
    writeLogging("Robot %d claimed" % robot)
    robotClaimed[robot] = True
    robotButtons[robot][INDEX_CLAIM   ].config(text = CMD_RELEASE)
    robotButtons[robot][INDEX_SELECT  ].config(state = NORMAL)
    robotButtons[robot][INDEX_SOFTWARE].config(state = NORMAL)
    robotButtons[robot][INDEX_REBOOT  ].config(state = NORMAL)
    u.sendTo(robot, "setClaim " + u.hostname)

    nrRobotsSelected = getNrRobotsSelected()
    if demoModeActive == True and nrRobotsSelected == 2:
        robotButtons[robot][INDEX_SELECT].config(text = CMD_SELECT)
        robotFrame[robot].config(bg=COLOR_DESELECTED)
    else:    
        robotButtons[robot][INDEX_SELECT].config(text = CMD_DESELECT)
        robotSelected[robot] = True
        robotFrame[robot].config(bg=COLOR_CLAIMED)
    enableDisableDemoButtons()

def releaseRobot(robot):
    writeLogging("Robot %d released" % robot)
    if robotStarted[robot]:
        stopRobotSoftware(robot)
    robotClaimed[robot] = False
    robotSelected[robot] = False
    robotButtons[robot][INDEX_CLAIM   ].config(text = CMD_CLAIM)
    robotButtons[robot][INDEX_SELECT  ].config(text = CMD_SELECT)
    robotButtons[robot][INDEX_SELECT  ].config(state = DISABLED)
    robotButtons[robot][INDEX_SOFTWARE].config(state = DISABLED)
    robotButtons[robot][INDEX_REBOOT  ].config(state = DISABLED)
    u.sendTo(robot, "releaseClaim " + u.hostname)
    robotFrame[robot].config(bg=DefaultColor)
    enableDisableDemoButtons()

def selectRobot(robot, select, update):
    if select == True:
        nrRobotsSelected = getNrRobotsSelected()
        if demoModeActive == True and nrRobotsSelected == 2:
            tkMessageBox.showerror("Precondition Error", "Not allowed to select more than 2 robots in Demo Mode")
        else:
            writeLogging("Robot %d selected" % robot)
            robotSelected[robot] = True
            robotButtons[robot][INDEX_SELECT].config(text = CMD_DESELECT)
            robotFrame[robot].config(bg=COLOR_CLAIMED)
    else:
        writeLogging("Robot %d deselected" % robot)
        robotSelected[robot] = False
        robotButtons[robot][INDEX_SELECT].config(text = CMD_SELECT)
        if robotClaimed[robot] == True:
            robotFrame[robot].config(bg=COLOR_DESELECTED)
    if update == True:
        enableDisableDemoButtons()

def startRobotSoftware(robot):
    robotStarted[robot] = True
    robotButtons[robot][INDEX_SOFTWARE].config(text = CMD_STOP)
    u.sendTo(robot, "jobStart robotReal")
    writeLogging("Software robot %d started" % robot)

def stopRobotSoftware(robot):
    robotStarted[robot] = False
    robotButtons[robot][INDEX_SOFTWARE].config(text = CMD_START)
    u.sendTo(robot, "jobStop robotReal")
    writeLogging("Software robot %d stopped" % robot)

def toggleRobotSoftware(robot):
    if robotStarted[robot]:
        stopRobotSoftware(robot)
    else:
        startRobotSoftware(robot)

def rebootRobot(robot):
    writeLogging("Robot %d rebooted" % robot)
    u.sendTo(robot, "restartSw")

def parkRobots():
    writeLogging("Robots moving to prepare position")
    xList = [ 6, -6, -6,  6]
    yList = [ 4,  4, -8, -8]
    x = xList[parkOrientation]
    y = yList[parkOrientation]
    for robot in robots:
        # regardless of claim, just send
        u.sendTo(robot, "scenario park %d %d" % (x, y))

def prepareRobots():
    writeLogging("Robots moving to prepare position")
    for robot in robots:
        # regardless of claim, just send
        u.sendTo(robot, "scenario matchPrepare")

def toggleCamera():
    global cameraEnabled
    if cameraEnabled:
        writeLogging("Camera disabled")
        cameraEnabled = False
        buttonDemoMode11.config(text = CMD_ENABLE_CAMERA)
    else:
        writeLogging("Camera enabled")
        cameraEnabled = True
        buttonDemoMode11.config(text = CMD_DISABLE_CAMERA)

def toggleJoystick():
    global joystickEnabled
    if joystickEnabled:
        writeLogging("Joystick disabled")
        joystickEnabled = False
        buttonDemoMode12.config(text = CMD_ENABLE_JOYSTICK)
    else:
        robot = getFirstSelectedRobot()
        if robot == None:
            tkMessageBox.showerror("Precondition Error", "No robot selected")
        else:
            writeLogging("Joystick enabled for robot %d" % robot)
            joystickEnabled = True
            buttonDemoMode12.config(text = CMD_DISABLE_JOYSTICK)
            rospy.init_node('commandGUI', anonymous=True)
            thread.start_new_thread( processJoystickInputInNewThread, (robot,) )
    
def toggleKeyboard():
    global keyboardEnabled
    if keyboardEnabled:
        writeLogging("Keyboard disabled")
        keyboardEnabled = False
        buttonDemoMode13.config(text = CMD_ENABLE_KEYBOARD)
        os.system("xset r on")
    else:
        robot = getFirstSelectedRobot()
        if robot == None:
            tkMessageBox.showerror("Precondition Error", "No robot selected")
        else:
            writeLogging("Keyboard enabled for robot %d" % robot)
            keyboardEnabled = True
            buttonDemoMode13.config(text = CMD_DISABLE_KEYBOARD)
            os.system("xset r off")
            thread.start_new_thread( processKeyboardInputInNewThread, (robot,) )

def toggleParkPosition():
    global parkOrientation
    parkOrientation = (parkOrientation + 1) % len(PARK_IMAGES)
    buttonMatchMode4.config(image = PARK_IMAGES[parkOrientation])

def getAllSelectedRobots():
    return [n for n in range(1, NR_OF_ROBOTS+1) if robotSelected[n]]
            
def getFirstSelectedRobot():
    robots = range(1, NR_OF_ROBOTS+1)
    for robot in robots:
        if robotSelected[robot]:
            return robot
    return None

def getNrRobotsSelected():
    return(len(getAllSelectedRobots()))
#    nrRobotsSelected = 0
#    robots = range(1, NR_OF_ROBOTS+1)
#    for robot in robots:
#        if robotSelected[robot]:
#            nrRobotsSelected += 1
#    return nrRobotsSelected

def getNrRobotsStarted():
    l = [n for n in range(1, NR_OF_ROBOTS+1) if robotStarted[n]]
    return( len(l))

def actionButtonPressed(cmd):
    goHomeSequence = [
        "target 0 0 0",
        SLEEP_10_SECONDS
    ]

    danceSequence = [
        "speed 0 0 2 6",
        SLEEP_7_SECONDS,
        "speed 0 0 -2 6",
        SLEEP_7_SECONDS,
        "speed 0 0.5 0 4",
        SLEEP_5_SECONDS,
        "speed 0 -0.5 0 8",
        SLEEP_9_SECONDS,
        "speed 0 0.5 0 4",
        SLEEP_5_SECONDS,
        "speed 0.5 0 0 4",
        SLEEP_5_SECONDS,
        "speed -0.5 0 0 8",
        SLEEP_9_SECONDS,
        "speed 0.5 0 0 4"
    ]

    makeSquareSequence = [
        "target 3 5 1.57",
        SLEEP_10_SECONDS,
        "target -3 5 3.14",
        SLEEP_10_SECONDS,
        "target -3 -5 4.71",
        SLEEP_10_SECONDS,
        "target 3 -5 0",
        SLEEP_10_SECONDS,
        "target 3 5 1.57",
        SLEEP_10_SECONDS,
        "target -3 5 3.14",
        SLEEP_10_SECONDS,
        "target -3 -5 4.71",
        SLEEP_10_SECONDS,
        "target 3 -5 0"
    ]
    if sequenceRunning == False:
        if cmd == CMD_GO_HOME:
            writeLogging("Go To Home Position")
            executeActionSequence(goHomeSequence)
        elif cmd == CMD_MAKE_SQUARE:
            writeLogging("Make Square")
            executeActionSequence(makeSquareSequence)
        elif cmd == CMD_DANCE:
            writeLogging("Dance")
            executeActionSequence(danceSequence)
        elif cmd == CMD_GET_BALL:
            executeAction("action getball")
        elif cmd == CMD_SHOOT_AT_GOAL:
            executeAction("action shoot")
        elif cmd == CMD_GOALKEEPER:
            executeAction("action goalie")
    else:
        tkMessageBox.showerror("Precondition Error", "Sequence currently being executed, use STOP button first")

def changeDrivingSpeed(speed):
    global drivingSpeed
    drivingSpeed = float(speed)
    writeLogging("Driving speed changed to %f" % drivingSpeed)

def changeKickPower(power):
    global kickPower
    kickPower = float(power)
    writeLogging("Kick power changed to %f" % kickPower)

def clearLogging():
    textLogging.delete(1.0, END)

def writeLogging(s):
    line = s + "\n"
    textLogging.insert(END, line)
    textLogging.see(END)
    localTime = time.asctime( time.localtime(time.time()))
    line += (localTime + ": ")
    loggingFile.write(line)

def showLoggingMenu(event):
    loggingMenu.tk_popup(event.x_root, event.y_root, 0)

def scrollbarLoggingPressed(text, step, what):
    '''
    print "Scrollbar arrow pressed: %s %s %s" % (text, step, what)
    if step == "1":
        if what == "pages":
            print "Scrolling down 1 page"
        if what == "units":
            print "Scrolling down 1 line"
    else:
        if what == "pages":
            print "Scrolling up 1 page"
        if what == "units":
            print "Scrolling up 1 line"
    '''
    pass

def toggleLogging():
    global loggingShown
    if loggingShown == True:
        loggingShown = False
        loggingMenu.entryconfig(0, label = "Show")
        frameLogging.pack_forget()
    else:
        loggingShown = True
        loggingMenu.entryconfig(0, label = "Hide")
        frameLogging.pack(side = BOTTOM)

def periodKeyPressed(event):
    global speedX
    global changeSpeedX
    if keyboardEnabled:
        speedX = drivingSpeed
        changeSpeedX = True

def periodKeyReleased(event):
    global speedX
    global changeSpeedX
    if keyboardEnabled:
        speedX = 0.0
        changeSpeedX = False

def commaKeyPressed(event):
    global speedX
    global changeSpeedX
    if keyboardEnabled:
        speedX = -drivingSpeed
        changeSpeedX = True

def commaKeyReleased(event):
    global speedX
    global changeSpeedX
    if keyboardEnabled:
        speedX = 0.0
        changeSpeedX = False

def upKeyPressed(event):
    global speedY
    global changeSpeedY
    if keyboardEnabled:
        speedY = drivingSpeed
        changeSpeedY = True

def upKeyReleased(event):
    global speedY
    global changeSpeedY
    if keyboardEnabled:
        speedY = 0.0
        changeSpeedY = False

def downKeyPressed(event):
    global speedY
    global changeSpeedY
    if keyboardEnabled:
        speedY = -drivingSpeed
        changeSpeedY = True

def downKeyReleased(event):
    global speedY
    global changeSpeedY
    if keyboardEnabled:
        speedY = 0.0
        changeSpeedY = False

def leftKeyPressed(event):
    global speedPhi
    global changeSpeedPhi
    if keyboardEnabled:
        speedPhi = drivingSpeed * 2.0
        changeSpeedPhi = True

def leftKeyReleased(event):
    global speedPhi
    global changeSpeedPhi
    if keyboardEnabled:
        speedPhi = 0.0
        changeSpeedPhi = False

def rightKeyPressed(event):
    global speedPhi
    global changeSpeedPhi
    if keyboardEnabled:
        speedPhi = -drivingSpeed * 2.0
        changeSpeedPhi = True

def rightKeyReleased(event):
    global speedPhi
    global changeSpeedPhi
    if keyboardEnabled:
        speedPhi = 0.0
        changeSpeedPhi = False

def enterKeyPressed(event):
    tkMessageBox.showwarning( "Warning", "Enter key pressed" )

def spaceBarPressed(event):
    global kickBall
    if keyboardEnabled:
        kickBall = True

def quitProgram():
    os.system("xset r on")
    nrRobotsStarted = getNrRobotsStarted()
    if nrRobotsStarted > 0:
       tkMessageBox.showwarning( "Warning", "Software still running on %d robot(s)" % nrRobotsStarted )
    root.quit()

def notImplementedYet(cmd):
    message = "%s has not been implemented yet" % cmd
    writeLogging(message)
    tkMessageBox.showerror("Precondition Error", message)

'''

Create Command GUI

'''
root = Tk()
root.title("Falcons command GUI")
root.resizable(width = FALSE, height = FALSE)
root.protocol("WM_DELETE_WINDOW", quitProgram)
img = Tkinter.Image("photo", file="/home/robocup/falcons/code/packages/coachCommands/gui.png")
root.tk.call('wm','iconphoto',root._w,img)

DefaultColor = root.cget("bg")

'''

Initialize local settings

'''
cameraEnabled   = False
demoModeActive  = False
joystickEnabled = False
keyboardEnabled = False
kickBall        = False
changeSpeed     = False
changeSpeedX    = False
changeSpeedY    = False
changeSpeedPhi  = False
parkAtLeftSide  = True
playGame        = False
sequenceRunning = False
testModeActive  = False
drivingSpeed    = 1.0
kickHeight      = 0.0
kickPower       = 40.0
speedX          = 0.0
speedY          = 0.0
speedPhi        = 0.0

'''

Create images for buttons

'''
image = Image.open(GIF_LEFT_ARROW)
image = image.resize((ARROW_WIDTH, ARROW_HEIGHT))
image.save("temp.gif")
lArrow = PhotoImage(file = "temp.gif")
image = Image.open(GIF_RIGHT_ARROW)
image = image.resize((ARROW_WIDTH, ARROW_HEIGHT))
image.save("temp.gif")
PARK_IMAGES = [PhotoImage(file = IMG_LOCATION + "park%d.png" % (r)).subsample(3) for r in range(1,5)]
image = Image.open(GIF_STOP_BUTTON)
image = image.resize((STOP_WIDTH, STOP_HEIGHT))
image.save("temp.gif")
stop = PhotoImage(file = "temp.gif")

image = Image.open(GIF_RESET_BUTTON)
image = image.resize((RESET_WIDTH, RESET_HEIGHT))
image.save("temp.gif")
reset = PhotoImage(file = "temp.gif")

os.remove("temp.gif")

'''

Create frame for selecting the main operating mode:
- Match Mode
- Demo Mode
- Test Mode

'''
frameMainMode = LabelFrame(root, text = "Mode")
frameMainMode.pack(fill = X, expand = True)

buttonMainMode1 = Button(frameMainMode, text = CMD_MATCH, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = selectMatchMode)
buttonMainMode1.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)
buttonMainMode1.config(relief = FLAT, background = COLOR_ACTIVATED, state = DISABLED)

buttonMainMode2 = Button(frameMainMode, text = CMD_DEMO, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = selectDemoMode)
buttonMainMode2.pack(side = LEFT, padx = PAD_X)

buttonMainMode3 = Button(frameMainMode, text = CMD_TEST, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = selectTestMode)
buttonMainMode3.pack(side = LEFT, padx = PAD_X)

buttonMainMode4 = Button(frameMainMode, text = CMD_SETTINGS, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = selectSettings)
buttonMainMode4.pack(side = LEFT, padx = PAD_X)

'''
buttonMainMode5 = Button(frameMainMode, image = stop, command = stopButtonPressed)
buttonMainMode5.pack(side = LEFT, padx = PAD_X, pady=PAD_Y)

buttonMainMode6 = Button(frameMainMode, image = reset, command = resetButtonPressed)
buttonMainMode6.pack(side = LEFT, padx = PAD_X, pady=PAD_Y)
'''

'''

For each robot create frame for selecting a robot action:
- Claim/Release
- Select/Unselect
- Start/Stop Software
- Reboot

This frame will always be visible

'''
robotButtons  = defaultdict(lambda: defaultdict(lambda: False))
robotClaimed  = defaultdict(lambda: False)
robotSelected = defaultdict(lambda: False)
robotStarted  = defaultdict(lambda: False)
robotFrame    = defaultdict()

frameRobots = PanedWindow(orient = HORIZONTAL, borderwidth = 2)
frameRobots.pack(fill = BOTH, expand = True)
robots = range(1, NR_OF_ROBOTS+1)
for robot in robots:
    f = LabelFrame(frameRobots, text = ("Robot %d" % (robot)))
    index = 0
    for cmd in [CMD_CLAIM, CMD_SELECT, CMD_START, CMD_REBOOT]:
        b=Button(f, text = cmd, width = ROBOT_BUTTON_WIDTH, height = BUTTON_HEIGHT)
        b.config(command = lambda robot = robot, cmd = cmd: robotButtonPressed(robot, cmd))
        b.pack(side = TOP, padx = PAD_X/2, pady = PAD_Y/3)
        if cmd != CMD_CLAIM:
            b.config(state = DISABLED)
        robotButtons[robot][index]=b
        index += 1
    f.pack()
    frameRobots.add(f)
    robotFrame[robot]=f

'''

Create frame for selecting match actions:
- Park
- Prepare
- Toggle park position

'''
frameMatchMode = LabelFrame(root, text = "Actions")

cmd = CMD_PARK
buttonMatchMode1=Button(frameMatchMode, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = parkRobots)
buttonMatchMode1.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

cmd = CMD_PREPARE
buttonMatchMode2=Button(frameMatchMode, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = prepareRobots)
buttonMatchMode2.pack(side = LEFT, padx = PAD_X)

buttonMatchMode4 = Button(frameMatchMode, image = PARK_IMAGES[0], command = toggleParkPosition)
buttonMatchMode4.pack(side = LEFT, padx = PAD_X)

frameMatchMode.pack(side = TOP, fill = X, expand = True)

'''

Create frame for selecting single robot actions:
- Enable/Disable Camera View
- Enable/Disable Joystick Usage
- Enable/Disable Keyboard Usage

'''
frameDemoMode1 = LabelFrame(root, text = "1 Robot")

cmd = CMD_ENABLE_CAMERA
buttonDemoMode11 = Button(frameDemoMode1, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: notImplementedYet(cmd))
#TODO buttonDemoMode11.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

buttonDemoMode12 = Button(frameDemoMode1, text = CMD_ENABLE_JOYSTICK, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = toggleJoystick)
buttonDemoMode12.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

buttonDemoMode13 = Button(frameDemoMode1, text = CMD_ENABLE_KEYBOARD, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = toggleKeyboard)
buttonDemoMode13.pack(side = LEFT, padx = PAD_X)

'''

Create frame for selecting single robot autonomous actions:
- Roles
- Behavior
- Action

'''
frameDemoMode2 = LabelFrame(root, text = "1 Robot Autonomous")

cmd = "Penalty"
buttonDemoMode21 = Button(frameDemoMode2, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: notImplementedYet(cmd))
buttonDemoMode21.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

cmd = "Goalkeeper"
buttonDemoMode22 = Button(frameDemoMode2, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: notImplementedYet(cmd))
buttonDemoMode22.pack(side = LEFT, padx = PAD_X)

cmd = "Attacker"
buttonDemoMode23 = Button(frameDemoMode2, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: notImplementedYet(cmd))
buttonDemoMode23.pack(side = LEFT, padx = PAD_X)

cmd = "Defender"
buttonDemoMode24 = Button(frameDemoMode2, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: notImplementedYet(cmd))
buttonDemoMode24.pack(side = LEFT, padx = PAD_X)

cmd = "Move"
buttonDemoMode25 = Button(frameDemoMode2, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: notImplementedYet(cmd))
# TODO buttonDemoMode25.pack(side = LEFT, padx = PAD_X)

'''

Create frame for selecting dual robot autonomous actions:
- Passing Ball
- Free Play (2 vs. 2)

'''
frameDemoMode3 = LabelFrame(root, text = "2 Robots Autonomous")

cmd = "Passing Ball"
buttonDemoMode31 = Button(frameDemoMode3, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: notImplementedYet(cmd))
buttonDemoMode31.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

cmd = "Free Play"
buttonDemoMode32 = Button(frameDemoMode3, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: notImplementedYet(cmd))
buttonDemoMode32.pack(side = LEFT, padx = PAD_X)

'''

Create frame for selecting test sequences:
- Go To Home Position
- Make Square
- Dance

'''
frameTestMode1 = LabelFrame(root, text = "Sequences")

cmd = CMD_GO_HOME
buttonTestMode11=Button(frameTestMode1, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: actionButtonPressed(cmd))
buttonTestMode11.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

cmd = CMD_MAKE_SQUARE
buttonTestMode12=Button(frameTestMode1, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: actionButtonPressed(cmd))
buttonTestMode12.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

cmd = CMD_DANCE
buttonTestMode13=Button(frameTestMode1, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: actionButtonPressed(cmd))
buttonTestMode13.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

buttonTestMode14 = Button(frameTestMode1, image = stop, command = stopButtonPressed)
buttonTestMode14.pack(side = LEFT, padx = PAD_X, pady=PAD_Y)

'''

Create frame for selecting test actions:
- Get Ball
- Shoot Ball at Goal
- Goalkeeper

'''
frameTestMode2 = LabelFrame(root, text = "Actions")

cmd = CMD_GET_BALL
buttonTestMode21=Button(frameTestMode2, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: actionButtonPressed(cmd))
#buttonTestMode21.grid(row = 0, column = 0, padx = PAD_X, pady = PAD_Y, sticky = W)
buttonTestMode21.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

cmd = CMD_SHOOT_AT_GOAL
buttonTestMode22=Button(frameTestMode2, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: actionButtonPressed(cmd))
#buttonTestMode22.grid(row = 0, column = 1, padx = PAD_X, pady = PAD_Y, sticky = W)
buttonTestMode22.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

cmd = CMD_GOALKEEPER
buttonTestMode23=Button(frameTestMode2, text = cmd, width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = lambda cmd = cmd: actionButtonPressed(cmd))
#buttonTestMode23.grid(row = 0, column = 2, padx = PAD_X, pady = PAD_Y, sticky = W)
buttonTestMode23.pack(side = LEFT, padx = PAD_X, pady = PAD_Y)

'''

Create frame for free format input

'''
frameTestMode3 = LabelFrame(root, text = "Free format action")
buttonTestMode3=Button(frameTestMode3, text = "Execute", width = BUTTON_WIDTH, height = BUTTON_HEIGHT, command = (lambda: executeAction(txtTestMode.get().strip())))
buttonTestMode3.grid(row=1, column=0, padx = PAD_X, pady = PAD_Y)
txtTestMode = Entry(frameTestMode3, width = 40)
txtTestMode.grid(row = 1, column=1)

'''

Create frame for selecting speed/power settings:
- Driving Speed
- Kick Power

'''
frameSettings2 = LabelFrame(root, text = "Speed/Power Settings")

labelSettings21 = Label(frameSettings2, text = "Driving Speed" )
scaleSettings21 = Scale(frameSettings2, orient = HORIZONTAL, length = 100, from_ = 1.0, to = 4.0, resolution = 0.1, command = changeDrivingSpeed)
scaleSettings21.set(drivingSpeed)
labelSettings21.grid(row = 0, column = 0, padx = PAD_X, pady = 0, sticky = W)
scaleSettings21.grid(row = 0, column = 1, padx = PAD_X, pady = 0, sticky = W)

labelSettings22 = Label(frameSettings2, text = "Kick Power" )
scaleSettings22 = Scale(frameSettings2, orient = HORIZONTAL, length = 100, from_ = 0, to = 60, resolution = 1, command = changeKickPower)
scaleSettings22.set(kickPower)
labelSettings22.grid(row = 1, column = 0, padx = PAD_X, pady = 0, sticky = W)
scaleSettings22.grid(row = 1, column = 1, padx = PAD_X, pady = 0, sticky = W)

'''

Create frame for logging

'''
frameLogging = Frame(root)
textLogging = Text(frameLogging, width = LOGGING_WIDTH, height = LOGGING_HEIGHT)
scrollbarLogging = Scrollbar(frameLogging, command=scrollbarLoggingPressed)
scrollbarLogging.pack(side=RIGHT, fill=Y)
textLogging.config(yscrollcommand=scrollbarLogging.set)
textLogging.bind("<Button-3>", showLoggingMenu)
textLogging.pack(side = LEFT)
frameLogging.pack(side = BOTTOM)

'''

Create menu bar

'''
menubar = Menu(root)

fileMenu = Menu(menubar, tearoff = 0)
fileMenu.add_command(label = CMD_MATCH, command = selectMatchMode)
fileMenu.add_command(label = CMD_DEMO, command = selectDemoMode)
fileMenu.add_command(label = CMD_TEST, command = selectTestMode)
fileMenu.add_command(label = CMD_SETTINGS, command = selectSettings)
fileMenu.add_separator()
fileMenu.add_command(label = "Exit", command = quitProgram)
menubar.add_cascade(label = "File", menu = fileMenu)

loggingMenu = Menu(menubar, tearoff = 0)
loggingMenu.add_command(label = "Hide", command = toggleLogging)
loggingMenu.add_command(label = "Clear", command = clearLogging)
menubar.add_cascade(label = "Logging", menu = loggingMenu)

helpMenu = Menu(menubar, tearoff = 0)
helpMenu.add_command(label = "About", command = showAbout)
menubar.add_cascade(label = "Help", menu = helpMenu)

root.config(menu = menubar)

'''

Create about window with Falcons logo

'''
windowAbout = Toplevel(root)
windowAbout.title("About")
windowAbout.protocol("WM_DELETE_WINDOW", closewindowAbout)
windowAbout.resizable(FALSE, FALSE)
falconsLogo = PhotoImage(file = GIF_FALCONS_LOGO)
canvas = Canvas(windowAbout, width = falconsLogo.width(), height = falconsLogo.height())
canvas.create_image(0, 0, image = falconsLogo, anchor = NW)
canvas.pack()
windowAbout.withdraw()
Label(windowAbout, text = "www.falcons-robocup.nl").pack(side = BOTTOM)
Label(windowAbout, text = "Python version: "+sys.version[:3]).pack(side = BOTTOM)
Label(windowAbout, text = "Tcl/Tk version: "+Tcl().eval("info patchlevel")).pack(side = BOTTOM)

root.bind('<KeyPress-period>', periodKeyPressed)
root.bind('<KeyRelease-period>', periodKeyReleased)
root.bind('<KeyPress-comma>', commaKeyPressed)
root.bind('<KeyRelease-comma>', commaKeyReleased)
root.bind('<KeyPress-Up>', upKeyPressed)
root.bind('<KeyRelease-Up>', upKeyReleased)
root.bind('<KeyPress-Down>', downKeyPressed)
root.bind('<KeyRelease-Down>', downKeyReleased)
root.bind('<KeyPress-Left>', leftKeyPressed)
root.bind('<KeyRelease-Left>', leftKeyReleased)
root.bind('<KeyPress-Right>', rightKeyPressed)
root.bind('<KeyRelease-Right>', rightKeyReleased)
root.bind('<Return>', enterKeyPressed)
root.bind('<space>', spaceBarPressed)

'''

Set up the UDP interface

'''
u = udpInterface()

'''

Open logging file

'''
loggingFileName = newest_logdir()+"/"+strftime("%Y%m%d_%H%M%S_CommandGUI.txt", localtime())
print "Logging file: %s" % loggingFileName
loggingFile = open(loggingFileName, "w")
loggingShown = True

enableDisableDemoButtons()

root.mainloop()
