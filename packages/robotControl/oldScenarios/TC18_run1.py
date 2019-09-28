""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotActions import *
from interceptBall import zoneInterceptBall

# Script for Robocup World Championship 2018, tech challenge run 1
# Conditions:
#   two robots, wifi on, normal MSL field and lighting
#   4 randomly placed, colored obstacles -> TODO: it is not at all clear that vision will be robust for this!
#   normal ball and black/white official ball --> Input Andre: cannot handle black/white ball, stick to normal MSL ball for now
#
# Choose robot playing direction AFTER having analyzed obstacle positions.
# Robot will try choose goal post such that it is not hampered by obstacle, but suppose both goal posts are blocked by an obstacle ... then better to play at other goal!
#
# Roles are based on starting position: the one close to field center gets main role 1, other support role 2
# Starting orientation may be used to skip parts of sequence, to allow quick resets.
# 
# Tricks/steps to perform:
#   1: pass over the ground 3 times (both robots)
#   2: shoot in goal 3 times (only main robot)
#   3: shoot at pole over the ground 3 times (only main robot)
#   4: shoot at bar 3 times (require support robot to catch and pass back)
#   5: lob pass, 8m away from each other (require support robot to catch and pass back)
# Tricks 4 and 5 are considered 'advanced'.


# R prefix: which role
FIELD_SIZE_LARGE          = False # toggle to switch between two field sizes
FIELD_SIZE_Y              = 9 + 2 * FIELD_SIZE_LARGE
FIELD_SIZE_X              = 6 + 1 * FIELD_SIZE_LARGE
PENALTY_AREA_SIZE_Y       = 2.25 # same for large field
GOAL_WIDTH                = 2 + 0.4 * FIELD_SIZE_LARGE
GOAL_POST_WIDTH           = 0.125
INTERCEPT_RADIUS_SMALL    = 2.0
INTERCEPT_RADIUS_LARGE    = 6.0
FORWARD_PHI               = 0.5*pi
BACKWARD_PHI              = 1.5*pi
PENALTY_Y                 = FIELD_SIZE_Y - 3 # rules: 3m from the goal, regardless of size
R1_START_POS              = (0, 0, FORWARD_PHI)             # start position, easy for simulator but should not be needed
R2_START_POS              = (0, -PENALTY_Y, FORWARD_PHI)    # start position, easy for simulator but should not be needed
R1_SEARCH_BALL_POS        = (0, PENALTY_Y, FORWARD_PHI)
SLEEP_SETTLE              = 1.0
SLEEP_AFTER_KICK          = 2.0
NUM_DOUBLE_PASSES         = 2 # 3 passes is sufficient but with an even number the ball is back at role1; 2 means twice from r1 to r2 and back
NUM_SIMPLE_GOALS          = 3 
NUM_GOAL_POST_HITS        = 3 
NUM_GOAL_BAR_HITS         = 3 
SIMPLE_GOAL_POWER         = 50.0
GOAL_POST_HIT_POWER       = 45.0
GOAL_BAR_KICK_POWER       = 78.0 # tuned on robot 4 at locht, may 22 [MVEH]
GOAL_BAR_KICK_HEIGHT      = 150  # tuned on robot 4 at locht, may 22 [MVEH]
TOLERANCE_FINE_XY         =  0.01
TOLERANCE_FINE_PHI        =  0.005

# all local functions are prefixed with an underscore _
# to be able to filter them in robotLibrary scenario selector

###### utility functions ######

def _waitUntilBothActive():
    return # TODO: remove this dev shortcut
    tt = teamMembers()
    while len(tt) < 1:
        sleep(1)
        tt = teamMembers()

def _waitTeamHasBall():
    while not teamHasBall():
        sleep(1)
    
def _determineTrickSequence():
    # TODO based on starting orientation, skip some steps?
    # however, rules do not allow adding point from separate attempts, so this is not useful during techChallenge (only useful during dev / test / tuning)
    return [4] # dev/test/tuning
    # return [1, 2, 3, 4, 5] # real test
    
def _calcPassMainPos():
    # assume robot just got the ball - use its position and face into direction where supporter should stand
    return (-3, PENALTY_Y, BACKWARD_PHI)
    
def _calcPassSupportPos():
    # TODO: calculate, use obstacles, or even better: moveToFreeSpotAround (email Coen)
    return (-3, PENALTY_Y-3, FORWARD_PHI)

def _calcAdvancedSupportPos():
    # TODO: calculate, use obstacles, or even better: moveToFreeSpotAround (email Coen)
    return (0, -PENALTY_Y, FORWARD_PHI)

def _waitUntilPassSupportReady():
    friendPos = getPosition(teamMembers()[0])
    passSupportPos = _calcPassSupportPos()
    sz = (Position2D(*passSupportPos) - Position2D(*friendPos)).xy().size()
    while sz > 0.5:
        sleep(1)
        friendPos = getPosition(teamMembers()[0])
        sz = (Position2D(*passSupportPos) - Position2D(*friendPos)).xy().size()
    
def _calcSimpleGoalPos():
    # TODO avoid any obstacle nearby goal? NO, choose starting orientation such that penalty spot can be used
    return (0, PENALTY_Y, FORWARD_PHI)
    
def _calcGoalPostPos():
    # choose goalpost with no obstacle nearby 
    posX = 0.5 * GOAL_WIDTH + 0.5 * GOAL_POST_WIDTH
    posLeft = (-posX, FIELD_SIZE_Y)
    posRight = (posX, FIELD_SIZE_Y)
    chooseLeft = True
    try:
        obstLeft = findClosestObstacle(posLeft)[0]
        distLeftSq = (obstLeft[0] - posLeft[0])**2 + (obstLeft[1] - posLeft[1])**2
        obstRight = findClosestObstacle(posRight)[0]
        distRightSq = (obstRight[0] - posRight[0])**2 + (obstRight[1] - posRight[1])**2
        if distRightSq < distLeftSq:
            chooseLeft = False
    except:
        pass
    result = posRight
    if chooseLeft:
        result = posLeft
    return (result[0], result[1], FORWARD_PHI)
    
def _interceptWrapper(x, y, phi, radius=INTERCEPT_RADIUS_SMALL, repositionRadius=INTERCEPT_RADIUS_SMALL):
    # robust intercept wrapper
    # postcondition: robot has the ball
    # large intercept radius already helps a lot due to the fallback inside that function, 
    # but we should be careful that other robot does not decide to go into fallback and interfere!
    zoneInterceptBall(x, y, phi, radius, repositionRadius) 
    # fallback getBall (TODO also searchball? no, anticipate multiCam + it is unlikely that we let the ball slip away beyond vision)
    sleep(1)
    if not hasBall():
        getBall()
    
def _passToTeamMember():
    sleep(SLEEP_SETTLE)
    # use motionPlanning pass action to first teammember, not teamplay
    friendPos = getPosition(teamMembers()[0])
    passTo(friendPos[0], friendPos[1])
    sleep(SLEEP_AFTER_KICK)

def _receivePass(radius=INTERCEPT_RADIUS_SMALL):
    # use zoneInterceptBall on current position
    _interceptWrapper(*ownPosition(), radius=radius)
    # it exits only when robot has ball

def _safeMove(*target):
    # we need this wrapper because of the following limitations of pathPlanning:
    # 1. when robot has fetched the ball, but there is an obstacle closeby, then tokyoDrift rotation will likely cause a collision
    #    so we should first drive backwards / away from obstacle using robotspeed
    # 2. when there is an obstacle at the target position, it will push it aside, causing a collision
    #    so we move to target location (while avoiding obstacles, default behavior) but not as far as to bump into it
    #    by adjusting target such that it is a certain distance away from closest obstacle
    #
    # TODO
    move(*target)
    
###### role and trick distribution ######        

def TC18_run1():
    # initialize
    _waitUntilBothActive()
    trickSequence = _determineTrickSequence() # might skip parts of sequence based on starting orientation
    # TODO: disable goal area obstacle avoidance - we want to be able to get a ball from goal
    # TODO tuning: reduce rotation speed&acc with ball to prevent losing it while turning (ballHandlers are currently our weak point)
    # distribute roles based on proximity to center, assume one starts within 3m from it and the other further away
    if robotCloseBy(0,0,xyTol=3):
        _role1(trickSequence)
    else:
        _role2(trickSequence)

def _role1(trickSequence):
    # start of sequence: find and get ball
    _role1_prepare()
    # list trick functions
    tricks = [None, _role1_trick1, _role1_trick2, _role1_trick3, _role1_trick4, _role1_trick5] 
    # do tricks
    for trick in trickSequence:
        f = tricks[trick]
        if f != None:
            f()
        
def _role2(trickSequence):
    # start of sequence: move to prep position
    _role2_prepare()
    # list trick functions
    tricks = [None, _role2_trick1, None, None, _role2_support, _role2_support] 
    # do tricks
    for trick in trickSequence:
        f = tricks[trick]
        if f != None:
            f()

###### role 1 (main) trick implementations ######

def _role1_prepare():
    # no need to scan field for obstacles: assume we can see them using multiCam; if not, there are only a few points lost
    # start of sequence: find and get ball
    _safeMove(*R1_START_POS)
    _safeMove(*R1_SEARCH_BALL_POS)
    getBall()

def _role1_trick1():
    targetPos = _calcPassMainPos()
    _safeMove(*targetPos)
    _waitUntilPassSupportReady() # sync point
    # iterate
    for passes in range(NUM_DOUBLE_PASSES):
        _safeMove(*targetPos)
        _passToTeamMember()
        _receivePass() # can only exit when having ball

def _role1_trick2():
    _safeMove(*R1_SEARCH_BALL_POS)
    targetPos = _calcSimpleGoalPos()
    # iterate
    for it in range(NUM_SIMPLE_GOALS):
        log('performing skill: simple goal (%d/%d)' % (1+it, NUM_SIMPLE_GOALS), 0)
        _safeMove(*targetPos)
        safeKick(SIMPLE_GOAL_POWER)
        sleep(0.3) # brief sleep, require tuning - to prevent robot from immediately chasing ball after having kicked
        _interceptWrapper(*targetPos)

def _role1_trick3():
    # choose position to shoot from
    _safeMove(*R1_SEARCH_BALL_POS) # need to move back in order to make a good choice
    goalPostPos = _calcGoalPostPos()
    targetPos = list(goalPostPos) # list to allow modifying
    targetPos[1] = targetPos[1] - PENALTY_AREA_SIZE_Y - 0.30 # just outside penalty area
    # sub target to fly in from some direction, minimizing inaccuracies due to ballHandling
    subTargetPos = list(targetPos) # list to allow modifying
    subTargetPos[1] = subTargetPos[1] - 0.30 # reduce y a bit
    # iterate
    for it in range(NUM_GOAL_POST_HITS):
        log('performing skill: hit goal post (%d/%d)' % (1+it, NUM_GOAL_POST_HITS), 0)
        # no safe move, because there should not be obstacles near targetPos
        move(*subTargetPos) # coarse move
        sleep(0.5) # settle
        move(*targetPos, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI) # increase accuracy by moving a bit forward
        # abuse a PASS to kick softly after precise aiming ... 
        # (actually, we now seem to require a better test interface: to shootAt target with preset kick power)
        passTarget = list(goalPostPos)
        passTarget[1] += 1.0 # project target a bit further to make the ball roll a bit faster, to reduce sensitivity due to field curvature and ball imperfections
        passTo(passTarget[0], passTarget[1])
        sleep(0.3) # brief sleep, require tuning - to prevent robot from immediately chasing ball after having kicked
        _interceptWrapper(*subTargetPos)
        
def _role1_trick4():
    # choose position to shoot from
    # TODO: now assume we are already facing the goal to shoot on; should determine based on obstacles which goal to use
    _safeMove(*R1_SEARCH_BALL_POS) # need to move back in order to make a good choice
    targetPos = list(R1_SEARCH_BALL_POS) # list to allow modifying
    targetPos[0] = 0 # center of goal # TODO: might be an obstacle there, determine position dynamically using a helper function similar to goalPostPos
    targetPos[1] = FIELD_SIZE_Y - PENALTY_AREA_SIZE_Y - 0.30 # just outside penalty area
    # sub target to fly in from some direction, minimizing inaccuracies due to ballHandling
    subTargetPos = list(targetPos) # list to allow modifying
    subTargetPos[1] = subTargetPos[1] - 0.30 # reduce y a bit
    # iterate
    for it in range(NUM_GOAL_BAR_HITS):
        log('performing skill: hit goal bar (%d/%d)' % (1+it, NUM_GOAL_BAR_HITS), 0)
        move(*subTargetPos) # coarse move
        sleep(0.5) # settle
        move(*targetPos, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI) # increase accuracy by moving a bit forward
        safeKick(GOAL_BAR_KICK_POWER, GOAL_BAR_KICK_HEIGHT)  # TODO: tuned on robot 4, verify before tech challenge!
        speed(1, 0, 0, 1) # move 1 meter to the right to avoid the bouncing ball; assume no obstacles nearby (that's why we choose this position)move 1 meter to the right
        sleep(2) # wait for ball to bounce passed us
        _interceptWrapper(*subTargetPos) # support should get the ball and pass it to us

def _role1_trick5():
    pass
    #targetPos = calcAdvancedMainPos()
    # TODO migrate from last year
    
###### role 2 (support) trick implementations ######

def _role2_prepare():
    # no need to scan field for obstacles: assume we can see them using multiCam; if not, there are only a few points lost
    # start of sequence: wait at secondary start position
    _safeMove(*R2_START_POS) 
    _waitTeamHasBall()

def _role2_trick1():
    # choose a dynamic position closeby main robot but avoiding obstacle
    targetPos = _calcPassSupportPos()
    _safeMove(*targetPos) # sync point
    for passes in range(NUM_DOUBLE_PASSES):
        _receivePass() # can only exit when having ball
        _safeMove(*targetPos)
        _passToTeamMember()

def _role2_support():
    # support role for advanced tricks (4 and 5)
    # choose a dynamic position very far away from main robot
    targetPos = _calcAdvancedSupportPos()
    while True: # this continues indefinitely - end of sequence anyway
        _receivePass(radius=INTERCEPT_RADIUS_LARGE) # can only exit when having ball
        _safeMove(*targetPos)
        _passToTeamMember()

