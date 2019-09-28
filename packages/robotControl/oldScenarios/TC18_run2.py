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
#   NOTE: since the space between obstacles is small, and we get really close to the goal area, need to disable obstacle avoidance!

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
        disableBallHandlers()
        # in simulator releasing the ball does not work; teleport it manually? Then print target position
        if phi > 2:
            # looking backward, so moving back means moving forward in y 
            move(x, y+0.3,phi)
        else:
            move(x, y-0.3,phi)
        enableBallHandlers()        
        getBall()   
        
# phi: 0 = 2pi = -x, pi = +x, 1.57 = +y, 4.71 = -y
def TC18_run2():
    setConfig("PathPlanningNode", {"Obstacle_avoidance_enabled":False}) # since the space between obstacles is small, and we get really close to the goal area, need to disable obstacle avoidance!
    tpControl("disabled") # disable to ensure we don't drive out of the penalty area after 10 s (ruleAvoidAreas); make sure teamplay is reset afterwards, otherwise it will not respond to refbox anymore
    phiY = 1.57
    phiMY = 4.71
    phiX = pi
    phiMX = 0    
    move(0, -6, phiY) 
    # ball is placed on 0,0, move close to it so we see it
    # teleport ball to 0,0 # manually type in simulator
    move(0, -1, phiY) 
    getBall()
    for parcours in range (0,3): # TODO: real run: 3 or 6 times, waiting for response rule committee;        
        # initially went diagonally in between obstacles (TC18_run2_diagonal.py); robot struggles to find a path in between --> go straight in between, also easier to ensure we don't dribble more than 3m
        TargetX = -1.5
        TargetY = 1.1 
        move(TargetX, TargetY, phiY)      
        moveBack()
        TargetX = -1.5
        TargetY = 2.9 
        move(TargetX, TargetY, phiY)      
        moveBack()
        for i in range (0,3):        
            TargetX = TargetX * -1
            move(TargetX, TargetY, phiY)
            moveBack()
            TargetY = TargetY + 1.8
            move(TargetX, TargetY, phiY)
            moveBack()
        # now we should be at 1.5, 8.3; move a bit away from the goal before crossing the field
        move(1.5, 8, phiX) 
        move(-1.5, 8, phiX) 
        moveBack() # now we dribble for 3.3 meter, assume nobody will notice
        TargetX = -1.5
        TargetY = 6.5
        move(TargetX , TargetY, phiMY)
        moveBack()
        for i in range (0,3):        
            TargetX = TargetX * -1
            move(TargetX, TargetY, phiMY)
            moveBack()
            TargetY = TargetY - 1.8
            move(TargetX, TargetY, phiMY)
            moveBack()
        move(-1.5, 1.1, phiY) # move to starting position
        moveBack()
    print "Step 1 parcours finished, starting step 2 backward dribble \n"
    move(-1.5, 0, phiY) # make sure we are angled correctly so we don't dribble backward out of the field
    ownPos = Position2D(*ownPosition())
    TargetX = ownPos.x
    TargetY = ownPos.y
    phi = ownPos.phi
    for i in range (0,3):        
        speed(0,-1,0,2)
        moveBack()
        sleep(1) # after getball wait to ensure we really have the ball before driving backwards 
    print "Step 2 backward dribble finished \n"
    tpControl("enabled") 


