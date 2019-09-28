""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
#from interceptBall import zoneInterceptBall

# Script for Robocup World Championship 2018, tech challenge run 2
# Conditions:
#   one robot, wifi on, normal MSL field and lighting
#   4 fixed obstacles
# Goal:
#   Step 1:  slalom dribble around them, without holding the ball for more than 3 meters
#   Step 2: backwards dribble for 1.5 meter, 3 times
#   
# pseudocode step 1:
#   Get ball
#   dribble to waypoint
#   stop + release ballhandlers
#   move back to release ball
#   get ball
#   move to next waypoint

# helperfunctions
def moveBack():
        ownPos = Position2D(*ownPosition())
        x = ownPos.x
        y = ownPos.y
        phi = ownPos.phi
        stop() # TODO: test if this is required
        disableBallHandlers()
        # in simulator releasing the ball does not work; teleport it manually? Then print target position
        move(x, y-0.2,phi)
        enableBallHandlers()        
        getBall()
        
# phi: 0 = 2pi = -x, pi = +x, 1.57 = +y, 4.71 = -y
def TC18_run2():
    phiY = 1.57
    phiMY = 4.71
    phiX = pi
    phiMX = 0    
    move(0, -6, phiY) # TODO: not needed in real version, just to avoid having to teleport in simulator
    ownPos = Position2D(*ownPosition())
    # expect own penalty marker, x = 0, y = -6 # Add verification step for real version?
    print ownPos # use log, example log('performing skill2 (slalom) %d times' % S2_NUM_PARCOURS, 0)  
    # ball is placed on 0,0, move close to it so we see it
    # teleport ball to 0,0 (not needed on field) # not possible in robotCLI, manually type in simulator
    move(0, -1, phiY) 
    getBall()
    for parcours in range (0,2): # real run: 6 times so range (0,6); for simulation this is enough        
        # TODO: dribbling too far, interrupt each movement with a "moveBack"; how?
        TargetX = -1.5
        TargetY = 2 
        move(TargetX, TargetY, phiY)      
        for i in range (0,3):        
            TargetX = TargetX * -1
            TargetY = TargetY + 1.8
            move(TargetX, TargetY, phiY)
        move(0, 8, phiX) # 8.25+ is goal area, forbidden to enter
        TargetX = -1.5
        TargetY = 7.4
        move(TargetX , TargetY, phiMY)
        for i in range (0,3):        
            TargetX = TargetX * -1
            TargetY = TargetY - 1.8
            move(TargetX, TargetY, phiMY)
        move(0, 0, phiMX)
    #moveBack() 
    print "Step 1 parcours finished, starting step 2 backward dribble \n"
    ownPos = Position2D(*ownPosition())
    TargetX = ownPos.x
    TargetY = ownPos.y
    phi = ownPos.phi
    for i in range (0,3):        
        TargetX = TargetX 
        TargetY = TargetY - 1.5
        move(TargetX, TargetY, phi)
        moveBack()
    print "Step 2 backward dribble finished \n"


