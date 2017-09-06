""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # Falcons match log visualizer library
# 2015-03-22 JFEI creation (borrowed some code from MKOE; use PGU)
# 2015-04-23 MKOE draw botnumber in visualizer
# 2015-06-27 JFEI add miniwindows at bottom
# 2015-06-30 JFEI add side text 
# 2015-07-04 JFEI pathplanning detailed drawing in main window (target, speed)
# 2015-07-07 MKOE implement teamplay-reasoning diagnostics info to be displayed
# 2016-02-** JFEI many changes, redesign for split diagnostics channel
# 2016-03-28 MKOE changed action number into 'human readable' action string
# 2016-03-28 MKOE expanded with reasoning task info
# 2016-03-31 MKOE alternate status display every half second for vision


import sys,os
import pygame
import copy
import socket
import collections
import datetime
import time
from pygame.locals import *
sys.path.append("/home/robocup/lib/pgu")
sys.path.append("/home/robocup/falcons/code/packages/coachCommands")
from hcolors import hcolors as colors
import txtlib
import operator
from pgu import gui
from math import cos, sin, sqrt
import subprocess
# Position2D, Velocity2D and their coordinate transformations
sys.path.append("/home/robocup/falcons/code/packages/facilities/common/src")
from FalconsCoordinates import *
from FalconsTrace import trace

#read/generate translationtable for actions numbers into strings
TURTLEROOT=os.environ['TURTLEROOT']
msgFilePath= TURTLEROOT + "/packages/facilities/rosMsgs/msg/t_action.msg"
with open( msgFilePath ) as msgFile:
   content = msgFile.readlines()

# lines look like: (without the first #)
#uint16 ADV_MOVE_TO_POS = 10020           #expects 6 or 3params: posX,posY,theta,velX,velY,velZ
actionTable={}

current_milli_time = lambda: int(round(time.time() * 1000))

for line in content:
   defLine=line.split()
   if len(defLine)>=4:
      if defLine[0][0] != "#":  #filter out comment lines
         actionName=defLine[1].split('_',1)[1]    #remove the ADV_  or BSC_ or any other prepending string before first underscore
         actionNumber=defLine[3]
         actionTable[actionNumber]=actionName


# constants
FPS                    = 12
R_botRadius            =  0.27
B_ballRadius           =  0.15
F_goalRadius           =  1.0
F_width                = 12.0
F_length               = 18.0
F_goalAreaOffset       =  0.75
F_goalDepth            =  0.6
F_penaltyAreaOffset    =  2.25
F_safetyBoundaryOffset =  1.50
F_lineThickness        =  0.125
NUM_ROBOTS             = 6
BALL_CONF_THRESHOLD    = 0.3
BATTERY_THRESHOLD      = 23.949
CPULOAD_THRESHOLD      = 3.0
NETLOAD_THRESHOLD      = 60.0 
TIMEOUT_EVENTS         = 10.0

RED     = (255,   0,   0)
GREEN   = (  0, 150,   0)
GREEN2  = (  0, 180,   0)
BLUE    = (  0,   0, 255)
CYAN    = (  0, 255, 255)
BLACK   = (  0,   0,   0)
YELLOW  = (255, 255,   0)
ORANGE  = (255, 165,   0)
MAGENTA = (255,   0, 255)
WHITE   = (255, 255, 255)

# the next is a quasi-constant: a matchlog is from the point of friendly team,
# either teamA or teamB
LEFT2RIGHT_TOGGLE = False


def update_screen_dimensions(zoomfactor):
    global ZOOM_MAIN, ZOOM_MINI
    global WIDTH_MINI, WIDTH_MAIN, WIDTH_SIDE, WIDTH_TOTAL
    global HEIGHT_MINI, HEIGHT_MAIN, HEIGHT_TOP, HEIGHT_TOTAL
    ZOOM_MAIN              = float(zoomfactor) # 36 is maximum for small screen JFEI
    ZOOM_MINI              = ZOOM_MAIN / NUM_ROBOTS
    WIDTH_MINI             = int(140.0/38*ZOOM_MAIN)
    WIDTH_MAIN             = WIDTH_MINI * NUM_ROBOTS # width of main field (left to right on screen), equals window width
    WIDTH_SIDE             = int(WIDTH_MINI * 1.6)
    WIDTH_TOTAL            = WIDTH_MAIN + WIDTH_SIDE 
    HEIGHT_MINI            = int(100.0/38*ZOOM_MAIN)
    HEIGHT_MAIN            = HEIGHT_MINI * NUM_ROBOTS
    HEIGHT_TOP             = 20
    HEIGHT_TOTAL           = HEIGHT_MINI + HEIGHT_MAIN + HEIGHT_TOP # height of complete window


def colorStr(formatstr, data, threshold, color=colors.FAIL, op=operator.lt):
    s = formatstr % (data)
    if op(data, threshold):
        s = color + s + colors.ENDC
    return s
        


class PlaybackControl(gui.Table):
    def __init__(self, matchlog, **params):
        gui.Table.__init__(self, **params)
        self.speed = 1.0
        self.t = 0.0
        self.paused = False
        def cb_slider_changed(slider):
            self.t = slider.value
        def cb_faster():
            self.speed *= 2.0
        def cb_slower():
            self.speed /= 2.0
        def cb_pause():
            self.paused = True
        def cb_play():
            self.paused = False
        fg = (255,255,255)
        self.tr()
        # first row of buttons: 
        #   - pause       ||
        #   - resume      >
        #   - slow down   <<
        #   - speedup     >>
        #   - time slider 
        btn = gui.Button("||")
        btn.connect(gui.CLICK, cb_pause)
        self.td(btn)
        btn = gui.Button(">")
        btn.connect(gui.CLICK, cb_play)
        self.td(btn)
        btn = gui.Button("<<")
        btn.connect(gui.CLICK, cb_slower)
        self.td(btn)
        btn = gui.Button(">>")
        btn.connect(gui.CLICK, cb_faster)
        self.td(btn)
        self.td(gui.Label(""))
        self.td(gui.Label(""))
        self.td(gui.Label(""))
        self.timeslider = gui.HSlider(0.0, 0.0, matchlog.elapsed, size=20, width=400, height=16, name='time', colspan=15)
        self.timeslider.connect(gui.CHANGE, cb_slider_changed, self.timeslider)
        self.td(self.timeslider)

    def update_time(self, dt):
        if not self.paused:
            self.t += self.speed * dt
            self.timeslider.value = self.t
        return self.t


class DetailControl(gui.Table):
    def __init__(self, **params):
        gui.Table.__init__(self, **params)
        self.verbose = False
        self.visionFrameSource = 0
        self.worldModel = True # TODO determine default based on simulation mode, need config management & logging it
        self.tr()
        # second row of buttons: 
        #   - verbose toggle
        def cb_verbose():
            self.verbose = not self.verbose
        btn = gui.Button("verbose")
        btn.connect(gui.CLICK, cb_verbose)


class InfoTextBox():
    """
    ASCII text box to the right side. 
    Allocates a few lines per robot which can be used to write interesting numbers/strings to.
    """
    def __init__(self, screen, robotnum, posx, posy):
        self.font = "liberationmono"
        self.hostname = socket.gethostname()
        self.fontsize = int(ZOOM_MAIN / 4.0)-1
        self.online = 0
        self.screen = screen
        self.robotnum = robotnum
        self.posx = posx
        self.posy = posy
        self.reset()
        self.altTextSelector=1
        self.lastAlternateTimestamp=current_milli_time()
        
    def reset(self):

        self.lines = {}
        self.dict = {}
        # template

        self.lines = []
        self.lines.append("ROBOT%d ($claimed): $inplay $branch" % (self.robotnum))
        self.lines.append("active: $activerobots")
        self.lines.append("health: bat=$battery proc=$proc cpu=$cpuload")
        self.lines.append("net   : bw=$netload ip=$ip loss=$netloss")
        self.lines.append("cmd   : refbox=$refbox coach=$command")
        #self.lines.append("teampl: task=$tptask")
        self.lines.append("teampl: s=$tpstate r=$tprole") 
        self.lines.append("teampl: b=$tpbehavior a=$tpaction") 
        self.lines.append("worldm: FR=$friends OBS=$obstacles BALL=$ballpossession")
        self.lines.append("perip : V=$encspeed cps=$compass bh=[$bhleft $bhright]")
        self.lines.append("vision: FPS=$visfps OBS=$visobst #BL=$visball PTS=$vispts")
        self.lines.append("vision: ACT=$visact AGE=$visage CONF=$visconf")

        assert(len(self.lines) <= 11) # more won't fit
        
    def setValue(self, key, value):
        self.dict[key] = value

    def __str__(self):
        # find out which lines to display 
        lines = self.lines

        # workaround for simulation mode: vision data is not generated, so remove those lines if data is not coming in
        if not self.dict.has_key("visfps"):
            lines = [s for s in lines if not s.startswith("vision: ")]
            lines = [s for s in lines if not s.startswith("perip : ")]
        # glue lines
        s = "\n".join(lines)
        # substitute given values
        for key in self.dict.keys():
            s = s.replace("$" + key, self.dict[key])
        return s
        
    def display(self):
        if self.online == 1:
            s = self.__str__()
            # make pixels
            text = txtlib.Text((WIDTH_SIDE, HEIGHT_TOTAL / (1.02 * NUM_ROBOTS)), self.font, font_size=self.fontsize)
            text.background_color = BLACK
            text.default_color = WHITE
            text.html(s)
            text.update()
            self.screen.blit(text.area, (self.posx, self.posy))
            return
        # offline!
        text = txtlib.Text((WIDTH_SIDE, HEIGHT_TOTAL / (1.02 * NUM_ROBOTS)), self.font, font_size=2*self.fontsize)
        text.background_color = BLACK
        text.default_color = RED
        if self.online == 2:
            text.html("R%d : LOST CONNECTION" % self.robotnum)
        else:
            text.html("R%d : OFFLINE" % self.robotnum)
        text.update()
        self.screen.blit(text.area, (self.posx, self.posy))

    def cbControl(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_control"""
        claimStr = ""
        if msg.claimedby == self.hostname:
            claimStr = colors.OKGREEN + msg.claimedby + colors.ENDC
        else:
            claimStr = colors.FAIL + msg.claimedby + colors.ENDC
        if len(msg.warning): 
            claimStr += colors.FAIL + "!" + colors.ENDC
        self.setValue("claimed", claimStr)
        self.setValue("command", msg.lastcommand)

    def cbActive(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_active"""
        # inplay / outofplay
        s = colors.OKGREEN + "inplay" + colors.ENDC
        if not msg.inplay:
            s = colors.FAIL + "OUTPLAY" + colors.ENDC
        self.setValue("inplay", s)
        # module activity (be robust for old versus new teamplay)
        try:
            s = colors.OKGREEN + "on" + colors.ENDC
            if not msg.reasoning:
                s = colors.FAIL + "off" + colors.ENDC
            self.setValue("activereas", s)
        except:
            pass
        try:
            s = colors.OKGREEN + "on" + colors.ENDC
            if not msg.actionHandler:
                s = colors.FAIL + "off" + colors.ENDC
            self.setValue("activeactions", s)
        except:
            pass
        s = colors.OKGREEN + "on" + colors.ENDC
        if not msg.pathPlanning:
            s = colors.FAIL + "off" + colors.ENDC
        self.setValue("activepp", s)
        s = colors.OKGREEN + "on" + colors.ENDC
        if not msg.shootPlanning:
            s = colors.FAIL + "off" + colors.ENDC
        self.setValue("activesp", s)
        
    def cbHealthSlow(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_health_slow"""
        branchStr = msg.gitBranch
        if len(msg.gitDirty):
            branchStr += colors.FAIL + "!" + colors.ENDC
        self.setValue("branch", branchStr)

    def cbHealthMid(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_health_mid"""
        self.setValue("ip", msg.ipAddress)
        procStr = ""
        if len(msg.deadProcesses):
            procStr = colors.FAIL + "NOK" + colors.ENDC
        else:
            procStr = colors.OKGREEN + "OK" + colors.ENDC
            if len(msg.badOutput):
                procStr += colors.FAIL + "!" + colors.ENDC
        self.setValue("proc", procStr)
        
    def cbHealthFast(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_health_fast"""
        self.setValue("netload", colorStr("%4.1fKB/s", msg.networkLoad, NETLOAD_THRESHOLD, op=operator.gt))
        self.setValue("cpuload", colorStr("%4.1f", msg.cpuLoad, CPULOAD_THRESHOLD, op=operator.gt))

    def cbWorldModel(self, msg, detailCtrl, visualizer = None, robotnum = 0):
        """Process messages from topics /teamA/robotN/g_diag_worldmodel"""
        self.setValue("friends", "%d" % len(msg.friends))
        actStr = ""
        for irobot in range(1,7):
            # also add self
            if (irobot in [r.id for r in msg.friends]) or (irobot == robotnum): 
                actStr += str(irobot)
            else:
                actStr += " "
            actStr += " "
        self.setValue("activerobots", actStr)
        self.setValue("obstacles", "%d" % len(msg.enemies))
        ballPossStr = colors.FAIL + "no ball" + colors.ENDC
        if msg.ballPresent:
            if msg.ballPossession.type == 1:
                ballPossStr = "on field"
            if msg.ballPossession.type == 2:
                ballPossStr = "at opponent"
            if msg.ballPossession.type == 3:
                ballPossStr = "at teammate %d" % msg.ballPossession.robotID
        self.setValue("ballpossession", ballPossStr)

    def cbHalMw(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_halmw"""
        self.setValue("bhleft", "%2d" % (int(msg.bh_left_angle)))
        self.setValue("bhright", "%2d" % (int(msg.bh_right_angle)))
        self.setValue("encspeed", "%4.1f" % (sqrt(msg.feedback_vx**2 + msg.feedback_vy**2)))
        self.setValue("battery", colorStr("%4.1f", msg.voltage, BATTERY_THRESHOLD))

    def cbCompass(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_compass"""
        self.setValue("compass", "%3d" % (int(msg.theta)))

    def cbRefbox(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_refbox"""
        self.setValue("refbox", msg.refboxCmd)

    def cbVision(self, msg, detailCtrl):
        """Process messages from topics /teamA/robotN/g_diag_vision"""
        self.setValue("visfps", colorStr("%4.1f", msg.fps, 5.0))
        self.setValue("vispts", colorStr("%3d", msg.linePoints, 50))
        self.setValue("visage", colorStr("%5.1f", msg.age, 3.0))
        self.setValue("visact", colorStr("%4.1f", msg.lastActive, 1.0, op=operator.gt))
        self.setValue("visobst", "%d" % len(msg.obstacles))
        self.setValue("visball", "%d" % len(msg.ballpos))
        self.setValue("visconf", colorStr("%.2f", msg.ownpos.confidence, 0.5))
        
    def cbTeamplay(self, msg):
        """Process messages from topics /teamA/robotN/g_diag_teamplay"""
        self.setValue("tpstate", msg.state)
        self.setValue("tpevent", msg.event)
        self.setValue("tprole", msg.role)
        self.setValue("tptask", msg.task)
        try:
            self.setValue("tpbehavior", msg.behavior)
        except:
            # not applicable for matchlogs with old teamplay
            pass
        try:
            self.setValue("tpaction", actionTable[msg.action])
        except:  #in case there is no actionTable entry, use the raw action number to display
            self.setValue("tpaction", msg.action)

class Field():
    """ 
    Visualization of a single field - there can be multiple.
    Offers primitives for drawing circle/line/rectangle/robot/ball.
    All coordinates are in ACS.
    """
    def __init__(self, parent, iscoach, robotnum, centerx, centery, width, height, zoom):
        self.parent = parent
        self.iscoach = iscoach
        self.robotNumber = robotnum
        self.zoom = zoom
        self.left2right = True
        self.width = int(width)
        self.height = int(height)
        self.centerXoffset = int(centerx)
        self.centerYoffset = int(centery)
        self.lineThickness = max(1, int(F_lineThickness * self.zoom))
        # center circle
        self.drawCircle(parent, (0,0), 2.0)
        # safety boundary
        x = F_width/2 + F_safetyBoundaryOffset
        y = F_length/2 + F_safetyBoundaryOffset
        self.drawRectangle((-x, -y), (x, y), BLACK)
        # field boundary
        x = F_width/2
        y = F_length/2
        self.drawRectangle((-x, -y), (x, y))
        # field half division line
        self.drawLine(parent, (-x, 0), (x, 0))
        # goals (outside play field, so where the ball needs to be)
        self.drawRectangle((-F_goalRadius, -y), (F_goalRadius, -y-F_goalDepth))
        self.drawRectangle((F_goalRadius, y), (-F_goalRadius, y+F_goalDepth))
        # goalie field inner zone
        x = F_goalRadius + F_goalAreaOffset
        self.drawRectangle((-x, -y), (x, -y+F_goalAreaOffset))
        self.drawRectangle((x, y), (-x, y-F_goalAreaOffset))
        # penalty zone
        x = F_goalRadius + F_penaltyAreaOffset
        self.drawRectangle((-x, -y), (x, -y+F_penaltyAreaOffset))
        self.drawRectangle((x, y), (-x, y-F_penaltyAreaOffset))
        # penalty positions
        self.drawCircle(parent, (0,6), 0.1)
        self.drawCircle(parent, (0,-6), 0.1)
        # ownpos needs to be persistent from vision to frontVision
        self.ownpos = None

    def field2pixelCoord(self, *args):
        if len(args) == 2:
            fieldx = args[0]
            fieldy = args[1]
            phi = 0.0
        elif len(args) == 1:
            # allow 'struct'
            try:
                fieldx = args[0].x
                fieldy = args[0].y
            except:
                # allow tuple
                try:
                    fieldx = args[0][0]
                    fieldy = args[0][1]
                except:
                    print args
                    raise Exception("could not parse args")
            try:
                phi = args[0].phi
            except:
                phi = 0.0
        else:
            raise Exception("missing arg")
        # transform FCS2ACS
        pos_fcs = Position2D(fieldx, fieldy, phi)
        l2r = self.left2right
        if LEFT2RIGHT_TOGGLE:
            l2r = not l2r
        pos_acs = pos_fcs.transform_fcs2acs(l2r)
        pixelx = self.centerXoffset + int(self.zoom * pos_acs.y)
        pixely = self.centerYoffset + int(self.zoom * pos_acs.x)
        return (pixelx, pixely)

    def drawRectangle(self, pos1, pos2, color=WHITE, lineThickness=None):
        if lineThickness == None:
            lineThickness = self.lineThickness
        lineThickness = max(1, int(lineThickness))
        tpos1 = self.field2pixelCoord(pos1)
        tpos2 = self.field2pixelCoord(pos2)
        w = max(tpos1[0], tpos2[0]) - min(tpos1[0], tpos2[0])
        h = max(tpos1[1], tpos2[1]) - min(tpos1[1], tpos2[1])
        rect = [min(tpos1[0], tpos2[0]), min(tpos1[1], tpos2[1]), w, h]
        pygame.draw.rect(self.parent, color, rect, lineThickness)

    # NOTE: the draw functions above are to be used statically, when drawing
    # the field background (lines). This is stored in surface 'grass' 
    # (via self.parent) and used to initialize each render iteration
    # HOWEVER bots, ball and such are dynamic in nature, so these do not 
    # use 'grass' but are to be put directly to screen.
    
    def drawLine(self, parent, pos1, pos2, color=WHITE, lineThickness=None):
        if lineThickness == None:
            lineThickness = self.lineThickness
        lineThickness = max(1, int(lineThickness))
        tpos1 = self.field2pixelCoord(pos1)
        tpos2 = self.field2pixelCoord(pos2)
        pygame.draw.lines(parent, color, True, [tpos1, tpos2], lineThickness)

    def drawPlayingDirection(self, parent):
        self.drawLine(parent, (2, 2), (2, 4), color=GREEN2, lineThickness=10)
        self.drawLine(parent, (3, 3), (2, 4), color=GREEN2, lineThickness=10)
        self.drawLine(parent, (1, 3), (2, 4), color=GREEN2, lineThickness=10)

    def drawCircle(self, parent, pos, radius, color=WHITE, lineThickness=None):
        """
        Inputs all in FCS. 
        """
        if lineThickness == None:
            lineThickness = self.lineThickness
        if lineThickness != 0:
            lineThickness = max(1, int(lineThickness))
        radius = max(int(self.zoom * radius), 1)
        centerpos = self.field2pixelCoord(pos)
        # hack
        if lineThickness > radius:
            lineThickness = radius
        pygame.draw.circle(parent, color, centerpos, radius, lineThickness)

    def drawBot(self, parent, pos, robotnumber=None, color=BLACK, drawmouth=True, radius=R_botRadius, directionLine=False, centered=False):
        # draw the direction as a line
        if directionLine:
            pos2d = Position2D(pos.x, pos.y, pos.phi)
            deltapos = Position2D(pos.x, pos.y, pos.phi)
            deltapos.x = cos(pos2d.phi) * 4.0
            deltapos.y = sin(pos2d.phi) * 4.0
            self.drawLine(self.parent, pos2d + deltapos, pos2d, color=MAGENTA, lineThickness=1)
        # don't use 'grass' as parent, because it is a static image
        # instead write directly to screen
        self.drawCircle(parent, pos, radius, color, lineThickness=0)
        if robotnumber != None:
            pixelpos = self.field2pixelCoord(pos)
            font = pygame.font.Font(None, 28)
            offsetx = -5
            offsety = -32
            if centered:
                offsetx = -5
                offsety = -8
                color = WHITE
            text = font.render(str(robotnumber), 1, color)
            parent.blit(text, (pixelpos[0] + offsetx, pixelpos[1] + offsety))
        # Draw orientation not as nice pacman with alpha, 
        # but for now by adding another smaller green circle. 
        # This overrides any white pixels below but whatever.
        # TODO: use pacman bitmap and rotate it with alpha (similar to oldvisualize)
        if drawmouth:
            mouthpos = copy.deepcopy(pos)
            mouthpos.x += cos(pos.phi) * 0.8 * R_botRadius
            mouthpos.y += sin(pos.phi) * 0.8 * R_botRadius
            self.drawCircle(parent, mouthpos, B_ballRadius, color=GREEN, lineThickness=0)

    def drawBall(self, parent, ballpos, robotnumber=None, color=YELLOW, showConf=False):
        # hack for transition from single ball to array of t_ball (keep supporting old msg)
        if isinstance(ballpos, list):
            for iball in range(len(ballpos)):
                ball = ballpos[iball]
                thiscolor = color
                # only best ball is drawn yellow, other balls are drawn orange
                if (iball > 0) or (ball.confidence < 0.5):
                    thiscolor = ORANGE
                self.drawBall(parent, ball, robotnumber, thiscolor, True)
            return
        # original drawBall
        pixelpos = self.field2pixelCoord(ballpos)
        if robotnumber != None:
            font = pygame.font.Font(None, 28)
            text = str(robotnumber)
            if showConf:
                text += " (%.2f)" % (ballpos.confidence)
            textRendered = font.render(text, 1, color)
            parent.blit(textRendered, (pixelpos[0]-5, pixelpos[1]-30))
        pygame.draw.circle(parent, color, pixelpos, int(B_ballRadius*self.zoom), 0)
        # draw the ball speed vector as a line w.r.t. ball
        if True:
            pos2d = Position2D(ballpos.x, ballpos.y, 0)
            deltapos = Position2D(ballpos.vx, ballpos.vy, 0)
            self.drawLine(self.parent, pos2d + deltapos, pos2d, color=YELLOW, lineThickness=1)

    def drawText(self, parent, fieldpos, textstr, color=BLUE, fontsize=28):
        # hack: fieldpos is in FCS, normally field2pixelCoord transforms to ACS
        # but for text we want to use relative ACS coordinates so disable transformation
        l2r_backup = self.left2right
        global LEFT2RIGHT_TOGGLE
        toggle_backup = LEFT2RIGHT_TOGGLE
        LEFT2RIGHT_TOGGLE = False
        self.left2right = True
        pixelpos = self.field2pixelCoord(fieldpos)
        self.left2right = l2r_backup
        LEFT2RIGHT_TOGGLE = toggle_backup
        font = pygame.font.Font(None, fontsize)
        renderedtext = font.render(textstr, 1, color)
        parent.blit(renderedtext, (pixelpos[0], pixelpos[1]))

    def cbWorldModel(self, msg, detailCtrl = None, visualizer = None, robotnum = 0):
        # obstacles in mainwindow
        if not self.iscoach:
            for robot in msg.enemies:
                visualizer.fields[0].drawBot(self.parent, robot, robotnum, color=RED, drawmouth=False, centered=True)
        # draw ball in mainwindow
        if robotnum and msg.ballPresent:
            try:
                visualizer.fields[0].drawBall(self.parent, msg.ballpos, robotnumber=robotnum)
            except:
                pass
        # return if in vision draw mode
        if detailCtrl != None and not detailCtrl.worldModel:
            return
        self.drawText(self.parent, (-9, -9), "%s@robot%d" % ("worldmodel", self.robotNumber), fontsize=int(ZOOM_MAIN / 3.6 * 1.4))
        if msg.ballPresent:
            # TODO: why can WM@coach state that ballPossession is FIELD, but not return a list with ball locations?
            # workaround: try catch (because positions are NaN)
            try:
                if robotnum:
                    self.drawBall(self.parent, msg.ballpos)
            except:
                pass
        # we do not visualize possession, instead see ASCII diagnostics
        # on coach?
        if self.iscoach:
            # friends are black and enemies are red
            for robot in msg.friends:
                self.drawBot(self.parent, robot, robot.id, color=BLACK, drawmouth=True, directionLine=True)
            #for robot in msg.enemies:
            #    self.drawBot(self.parent, robot, None, color=RED, drawmouth=False)
            # we do not use this anymore, instead draw obstacles based on per-robot knowledge in mainwindow, together with number
        else:
            # friends are blue and enemies are red
            for robot in msg.friends:
                self.drawBot(self.parent, robot, None, color=BLUE, drawmouth=False)
            for robot in msg.enemies:
                self.drawBot(self.parent, robot, None, color=RED, drawmouth=False)
            # our own position, but without orientation (too few pixels)
            self.drawBot(self.parent, msg.ownpos, None, color=BLACK, drawmouth=False, directionLine=True)
        # draw playing direction
        self.drawPlayingDirection(self.parent)

    def cbPathPlanning(self, msg, robotnumber):
        # don't draw target/subtarget if inactive
        if msg.active == False:
            return
        # subtarget as magenta ball (so small and without orientation)
        try:
            self.drawBall(self.parent, msg.subtarget, robotnumber, color=MAGENTA)
        except:
            pass
        # target as a magenta 'ghost' robot
        # (we could choose to not draw if close enough, but now the robot will
        #  be drawn on top of it anyway)
        self.drawBot(self.parent, msg.target, robotnumber, color=MAGENTA)

    def cbHealthSlow(self, msg):
        if "playingDirection = 1" in msg.globalConfig:
            self.left2right = False
        else:
            self.left2right = True
    
    def cbRefbox(self, msg, visualizer):
        trace("received refbox " + msg.refboxCmd)
        visualizer.lastRefboxCmd = msg.refboxCmd

    def cbVisionFrame(self, msg, robotnumber, detailCtrl, visualizer):
        if detailCtrl != None and (detailCtrl.visionFrameSource == 0):
            visualizer.visionFrameData = None
            return
        # only plot if source matches
        if robotnumber == detailCtrl.visionFrameSource:
            # I'm not too handy with frame buffering and transformations... so just dump the buffer to jpg file, then load
            trace("vision frame write")
            tmpfile = "/var/tmp/visionframe.jpg"
            # convert signed to unsigned bytes
            file(tmpfile, "w").write(bytearray([(256+v)%256 for v in msg.data]))
            trace("vision frame load")
            visualizer.visionFrameData = pygame.image.load(tmpfile)
            trace("vision frame done")
    
    def cbVision(self, msg, detailCtrl):
        # return if in worldModel draw mode (simulation; or user pressed 'w')
        if detailCtrl != None and detailCtrl.worldModel:
            return
        self.drawText(self.parent, (-9, -9), "%s@robot%d" % ("vision", self.robotNumber), fontsize=int(ZOOM_MAIN / 3.6 * 1.4))
        # own position with a cross
        ownpos_fcs = Position2D(msg.ownpos.x, msg.ownpos.y, msg.ownpos.phi)
        projected_pos = copy.copy(ownpos_fcs)
        deltapos = copy.copy(ownpos_fcs)
        crossscaling = 1.0
        deltapos.x = cos(ownpos_fcs.phi) * crossscaling
        deltapos.y = sin(ownpos_fcs.phi) * crossscaling
        self.drawLine(self.parent, ownpos_fcs + deltapos * 3.0, ownpos_fcs + deltapos * -1.0, color=MAGENTA, lineThickness=1)
        deltapos = copy.copy(ownpos_fcs)
        deltapos.x = cos(ownpos_fcs.phi + math.pi*0.5) * crossscaling
        deltapos.y = sin(ownpos_fcs.phi + math.pi*0.5) * crossscaling
        self.drawLine(self.parent, ownpos_fcs + deltapos, ownpos_fcs + deltapos * -1.0, color=MAGENTA, lineThickness=1)
        self.drawBot(self.parent, msg.ownpos, None, color=BLACK, drawmouth=False, radius=2*R_botRadius)
        # store ownpos so it can be used in processing other relative data (e.g. frontVision)
        self.ownpos = ownpos_fcs
        # balls
        for elem in msg.ballpos:
            if elem.confidence > BALL_CONF_THRESHOLD:
                pos = Position2D() # first in RCS
                angle_offset = math.pi * 0.0;
                pos.x = cos(elem.angle + angle_offset) * elem.radius
                pos.y = sin(elem.angle + angle_offset) * elem.radius
                # now transform to FCS
                pos = pos.transform_rcs2fcs(ownpos_fcs) 
                # draw ball on size of robot, too few pixels otherwise
                #self.drawBall(self.parent, pos)
                self.drawBot(self.parent, pos, None, color=YELLOW, drawmouth=False)
        # obstacles
        for elem in msg.obstacles:
            pos = Position2D() # first in RCS
            angle_offset = math.pi * 0.0;
            pos.x = cos(elem.angle + angle_offset) * elem.radius
            pos.y = sin(elem.angle + angle_offset) * elem.radius
            # now transform to FCS
            pos = pos.transform_rcs2fcs(ownpos_fcs) 
            self.drawBot(self.parent, pos, None, color=RED, drawmouth=False)
      
    def cbFrontVision(self, msg, detailCtrl):
        # return if in worldModel draw mode (simulation; or user pressed 'w')
        if detailCtrl != None and detailCtrl.worldModel:
            return
        # own position needs to have been set by normal vision
        if self.ownpos == None:
            return
        # balls
        cameraFrontOffset = 0.15
        cameraHeight = 0.38
        for elem in msg.balls:
            pos = Position2D()
            angle_offset = math.pi * 0.0;
            pos.x = cos(elem.angle + angle_offset) * elem.radius
            pos.y = cameraFrontOffset + elem.confidence # abuse confidence field as distance
            z = sin(elem.angle + angle_offset) * elem.radius + cameraHeight # unused until new 3D visualizer
            # now transform to FCS
            pos = pos.transform_rcs2fcs(self.ownpos) 
            # draw ball on size of robot, too few pixels otherwise
            self.drawBot(self.parent, pos, None, color=CYAN, drawmouth=False)
      
        
class Visualizer():
    def __init__(self, matchlog, bagged, zoom = "auto"):
        self.verbose = False
        self.lastRefboxCmd = ""
        self.visionFrameData = None
        # store matchlog
        self.bagged = bagged
        self.matchlog = matchlog
        # determine global zoom
        if zoom == "auto":
            zoom = 36.0
            screendims_str = subprocess.check_output("xrandr", stderr=subprocess.STDOUT)
            for line in screendims_str.splitlines():
                if "*" in line:
                    for word in line.split():
                        if "x" in word:
                            dims = [float(s) for s in word.split('x')]
                            zoom = min(dims) / 1024.0 * 50.0
        else:
            zoom = float(zoom)
        #print "using zoom factor: ", zoom
        update_screen_dimensions(zoom)
        # GUI init
        self.screen = pygame.display.set_mode((WIDTH_TOTAL,HEIGHT_TOTAL),SWSURFACE)
        self.form = gui.Form()
        self.app = gui.App()
        c = gui.Container(align=-1,valign=-1)
        # construct the slider and buttons to control playback
        if self.bagged:
            self.playbackCtrl = PlaybackControl(matchlog)
            c.add(self.playbackCtrl,0,0)
        # detailed info toggle buttons
        self.detailCtrl = DetailControl()
        c.add(self.detailCtrl,0,20)
        # "grass": green background with lines, which we store statically
        # and blit each render iteration
        self.grass = pygame.Surface((WIDTH_TOTAL, HEIGHT_TOTAL)) 
        self.grass.fill(GREEN)
        # main field (visualizing WorldModel @coach)
        self.fields = {}
        offset_y = HEIGHT_MAIN * 0.5 + HEIGHT_TOP
        self.fields[0] = Field(self.grass, True, 0, WIDTH_MAIN/2, offset_y, WIDTH_MAIN, HEIGHT_MAIN, ZOOM_MAIN)
        self.fields[0].parent = self.screen
        # minifields
        for robotnum in range(1,NUM_ROBOTS+1):
            centerx = WIDTH_MINI*0.5 + WIDTH_MINI*(robotnum-1)
            centery = HEIGHT_MINI*0.5 + HEIGHT_TOP + HEIGHT_MAIN
            self.fields[robotnum] = Field(self.grass, False, robotnum, centerx, centery, WIDTH_MINI, HEIGHT_MINI, ZOOM_MINI)
            self.fields[robotnum].parent = self.screen
        # event logging
        self.events = []
        # ASCII text boxes
        self.textBoxes = {}
        for robotnum in range(1,NUM_ROBOTS+1):
            posx = 10 + WIDTH_MAIN
            posy = 3 + HEIGHT_TOTAL * (robotnum-1) / (1.0 * NUM_ROBOTS)
            self.textBoxes[robotnum] = InfoTextBox(self.screen, robotnum, posx, posy)
        # install generic text callback in Matchlog, to dump to stdout (only if 'verbose')
        self.matchlog.genericCallback = self.dumpToStdout
        # install specific draw callbacks in Matchlog
        self.matchlog.drawCallbacks["/teamA/g_worldmodel"] = self.fields[0].cbWorldModel
        self.matchlog.drawCallbacks["/teamA/g_ana_online"] = self.cbOnline
        self.matchlog.drawCallbacks["/teamA/g_ana_error"] = self.cbError
        self.matchlog.drawCallbacks["/teamA/g_ana_info"] = self.cbInfo
        self.matchlog.drawCallbacks["/teamA/g_diag_refbox"] = (self.fields[0].cbRefbox, self)
        for robotnum in range(1,NUM_ROBOTS+1):
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_control" % robotnum] = self.textBoxes[robotnum].cbControl
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_active" % robotnum] = self.textBoxes[robotnum].cbActive
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_health_slow" % robotnum] = (self.textBoxes[robotnum].cbHealthSlow, self.fields[robotnum].cbHealthSlow)
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_health_mid" % robotnum] = self.textBoxes[robotnum].cbHealthMid
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_health_fast" % robotnum] = self.textBoxes[robotnum].cbHealthFast
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_pathpl" % robotnum] = (self.fields[0].cbPathPlanning, robotnum)
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_halmw" % robotnum] = (self.textBoxes[robotnum].cbHalMw)
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_compass" % robotnum] = (self.textBoxes[robotnum].cbCompass)
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_vision" % robotnum] = (self.fields[robotnum].cbVision, self.textBoxes[robotnum].cbVision, self.detailCtrl)
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_frontvision" % robotnum] = (self.fields[robotnum].cbFrontVision, self.detailCtrl)
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_teamplay" % robotnum] = self.textBoxes[robotnum].cbTeamplay
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_worldmodel" % robotnum] = (self.fields[robotnum].cbWorldModel, self.textBoxes[robotnum].cbWorldModel, self.detailCtrl, self, robotnum)
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_refbox" % robotnum] = (self.textBoxes[robotnum].cbRefbox)
            self.matchlog.drawCallbacks["/teamA/robot%d/g_diag_vision_rframe" % robotnum] = (self.fields[robotnum].cbVisionFrame, robotnum, self.detailCtrl, self)
        # done
        self.app.init(c)

    def cbOnline(self, msg):
        for robotnum in range(1,NUM_ROBOTS+1):
            # subtract one for the index
            self.textBoxes[robotnum].online = ord(msg.online[robotnum-1])

    def cbError(self, msg):
        # determine time of the event
        t = datetime.datetime.fromtimestamp(self.matchlog.t0 + self.currenttime)
        timestamp = (str(t).replace(' ', ','))
        timestr = "time=%s : " % (timestamp[11:21])    
        # always print to stdout
        s = timestr + msg.error
        print s.strip()
        # shorten uninteresting part of a possibly long string
        s = s.replace("/home/robocup/falcons/code/packages/", "")
        self.events.append((self.currenttime, s, 0))

    def cbInfo(self, msg):
        t = datetime.datetime.fromtimestamp(self.matchlog.t0 + self.currenttime)
        timestamp = (str(t).replace(' ', ','))
        timestr = "time=%s : " % (timestamp[11:21])    
        # always print to stdout
        s = timestr + msg.info
        print s.strip()
        # shorten uninteresting part of a possibly long string
        s = s.replace("/home/robocup/falcons/code/packages/", "")
        self.events.append((self.currenttime, s, 1))

    def drawEvents(self):
        # clean out old messages
        self.events = [e for e in self.events if e[0] + TIMEOUT_EVENTS > self.currenttime]
        # poor man scrolling text
        offset = 0
        fs = ZOOM_MAIN / 1.8
        ostep = fs * 0.013
        for e in self.events:
            fs = ZOOM_MAIN / 1.8
            if len(e[1]) > 90:
                fs *= 0.7
            self.fields[0].drawText(self.screen, (-5 + offset, -8), e[1], color=[RED, BLUE][e[2]], fontsize=int(fs))
            offset += ostep
    
    def dumpToStdout(self, topic, msg):
        if self.detailCtrl.verbose:
            print ""
            print "time = %6.2fs  topic = %s" % (self.currenttime, topic)
            print msg
            
    def run(self):
        clock = pygame.time.Clock()
        done = False
        self.currenttime = 0
        while not done:
            for e in pygame.event.get():
                if e.type is QUIT: 
                    done = True
                elif e.type is KEYDOWN and e.key == K_ESCAPE: 
                    done = True
                elif e.type is KEYDOWN and e.key == K_v: 
                    self.detailCtrl.verbose = not self.detailCtrl.verbose
                elif e.type is KEYDOWN and e.key in [K_0, K_1, K_2, K_3, K_4, K_5]: 
                    if e.key == K_0:
                        self.detailCtrl.visionFrameSource = 0
                    else:
                        self.detailCtrl.visionFrameSource = [K_0, K_1, K_2, K_3, K_4, K_5].index(e.key)
                elif e.type is KEYDOWN and e.key == K_f: 
                    global LEFT2RIGHT_TOGGLE
                    LEFT2RIGHT_TOGGLE = not LEFT2RIGHT_TOGGLE
                elif e.type is KEYDOWN and e.key == K_w: 
                    self.detailCtrl.worldModel = not self.detailCtrl.worldModel
                else:
                    self.app.event(e)
            # clear the screen and render game state
            dt = clock.tick(FPS)/1000.0
            self.screen.fill((0,0,0))
            # update time, only for bag playback
            if self.bagged:
                self.currenttime = self.playbackCtrl.update_time(dt)
                if self.currenttime > self.matchlog.elapsed:
                    done = True
            else:
                self.currenttime += dt
            self.render()
            self.app.paint()
            pygame.display.flip()
            
    def render(self):
        trace("render start %10.3f", self.currenttime)
        # draw static fields (green background and lines)
        self.screen.blit(self.grass, (0,0))
        t = datetime.datetime.fromtimestamp(self.matchlog.t0 + self.currenttime)
        timestamp = (str(t).replace(' ', ','))
        timestr = "time=%s" % (timestamp[11:21])
        try:
            timestr += " (@%4.1fx)" % (self.playbackCtrl.speed)
        except:
            pass
        timestr += "   press w to toggle vision/worldmodel"
        self.fields[0].drawText(self.screen, (-8, 0), timestr, color=BLACK, fontsize=int(ZOOM_MAIN / 2.0))
        self.fields[0].drawText(self.screen, (-7, 0), self.lastRefboxCmd, color=RED, fontsize=int(ZOOM_MAIN / 1.2))
        # trigger all draw callbacks
        trace("advancing ...")
        self.matchlog.advance(self.currenttime)
        # show error text
        self.drawEvents()
        # show visionFrameData
        if self.visionFrameData != None:
            self.screen.blit(self.visionFrameData, (0,20))
        # show ASCII text boxes
        for box in self.textBoxes.values():
            box.display()
        trace("render end")




