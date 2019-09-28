""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 from robotScenarioBase import *
from FalconsCoordinates import Vec2d, RobotPose
from math import pi
import logging


# Script for Robocup World Championship 2018, tech challenge run 1
# Conditions:
#   two robots, wifi on, normal MSL field and lighting
#   4 randomly placed, colored obstacles (not black!)
#   normal ball and black/white official ball 
#      --> Input Andre: cannot handle black/white ball, stick to normal MSL ball for now
#      (too bad, 30% of points cannot be attained)
#
# It is not really clear if we can choose playing direction based on obstacles positions.
# Rules suggest that robots are placed, then balls based on robots, finally obstacles.
# But thanks to the new heightmap (awesome suggestion by Coen) we should be OK.
#
# Roles are based on starting position: the one close to field center gets main role 1, other support role 2
# 
# Tricks/steps to perform:
#   1: pass over the ground 3 times (both robots)
#   2: shoot in goal 3 times (only main robot)
#   3: shoot at pole over the ground 3 times (only main robot)
#   4: shoot at bar 3 times (require support robot to catch and pass back)
#   5: lob pass, 8m away from each other (require support robot to catch and pass back)
# Tricks 4 and 5 are considered 'advanced'.


# R prefix: which role
TRICK_SEQUENCE            = [2, 3, 4, 5]
MAIN_PLAYER_ONLY          = False
SIMULATION                = True
FIELD_SIZE                = 1 # 0 is small (Canada), 1 is normal (Locht) and 2 is large (2018+)
FIELD_SIZE_Y              = [5.8, 9, 11][FIELD_SIZE]
FIELD_SIZE_X              = [5, 6, 7][FIELD_SIZE]
PENALTY_AREA_SIZE_Y       = 2.25 # same for large field
GOAL_WIDTH                = [2.42, 2.0, 2.4][FIELD_SIZE]
GOAL_POST_WIDTH           = [0.10, 0.125, 0.10][FIELD_SIZE]
INTERCEPT_RADIUS_SMALL    = 3.0
INTERCEPT_RADIUS_LARGE    = 6.0
FORWARD_PHI               = 0.5*pi
BACKWARD_PHI              = 1.5*pi
PENALTY_Y                 = FIELD_SIZE_Y - 3 # rules: 3m from the goal, regardless of size
R1_START_POS              = (0, 2*(FIELD_SIZE == 0))
R2_START_POS              = (0, -4)
LANE_X_POSITION           = 0.0 # calculated by _determineLaneX and used to override passing & lob bar x positions, avoiding obstacles as good as possible
R1_SEARCH_BALL_POS        = (0, PENALTY_Y)
SLEEP_SETTLE              = 1.0
SLEEP_AFTER_KICK          = 1.0 # large enough to not immediately chase the ball, but short enough to get moving to next position (see _waitUntilPassReceiverReady)
NUM_DOUBLE_PASSES         = 2 # 3 passes is sufficient but with an even number the ball is back at role1; 2 means twice from r1 to r2 and back
NUM_LOB_PASSES            = 5 - 3 * SIMULATION
NUM_SIMPLE_GOALS          = 4 - 2 * SIMULATION
NUM_GOAL_POST_HITS        = 5 - 3 * SIMULATION
NUM_GOAL_BAR_HITS         = 4 - 2 * SIMULATION
SIMPLE_GOAL_POWER         = 50.0 + (20.0 * SIMULATION)
GOAL_POST_HIT_POWER       = 45.0 + (8.0 * SIMULATION) # UNUSED -- we use motionPlanning pass action including its power setpoint calculation
GOAL_BAR_KICK_POWER       = 77.0 # tuned on robot 4 at Canada
GOAL_BAR_KICK_HEIGHT      = 150  # tuned on robot 4 at locht, may 22 [MVEH]
LOB_PASS_KICK_POWER       = 86.0
LOB_PASS_KICK_HEIGHT      = 200
TOLERANCE_FINE_XY         = 0.01
TOLERANCE_FINE_PHI        = 0.005
OBSTACLE_CLOSEBY_LIMIT    = 0.7

# all local functions are prefixed with an underscore _
# to be able to filter them in robotLibrary scenario selector

###### utility functions ######

def _log(s):
    logging.info(s)
    
def _waitUntilBothActive():
    if MAIN_PLAYER_ONLY:
        return
    tt = robot.teamMembers()
    while len(tt) < 1:
        sleep(1)
        tt = robot.teamMembers()

def _waitTeamHasBall():
    _log("waiting until team has the ball ...")
    while not robot.teamHasBall():
        sleep(1)
    _log("team has the ball")

def _ensureHasBall():
    if not robot.hasBall():
        _log("ensuring we have the ball")
        robot.getBall()

def _determineTrickSequence():
    # TODO based on starting orientation, skip some steps?
    # however, rules do not allow adding points from separate attempts, each attempt is a full restart
    # so this is not useful during techChallenge (only useful during dev / test / tuning)
    return TRICK_SEQUENCE

def _friendPosition():
    return robot.getPosition(robot.teamMembers()[0])

def _friendVelocity():
    return robot.getVelocity(robot.teamMembers()[0])

def _friendIsMoving():
    if MAIN_PLAYER_ONLY:
        return False
    return (_friendVelocity().xy().size() > 0.1)

def _calcPassMainPos():
    # assume robot just got the ball - use its position and face into direction where supporter should stand
    result = robot.getPosition()
    result.Rz = BACKWARD_PHI
    return (result.x, result.y, result.Rz)
    
def _calcPassSupportPos():
    # calculate based on friend position, rely on use moveToFreeSpotNearPosition
    result = _friendPosition() 
    result.y = -3 # position at other half
    result.Rz = FORWARD_PHI
    return (result.x, result.y, result.Rz)

def _calcAdvancedSupportPos():
    return (LANE_X_POSITION, -4, FORWARD_PHI)

def _determineLaneX():
    # sync: wait until both robots stand still
    # main robot on penalty position
    # support robot on (0,-4)
    # this should give a good view on where the obstacles are
    # the robot calling this function is already standing still
    # so the sync is achieved by waiting on the other
    global LANE_X_POSITION
    if MAIN_PLAYER_ONLY:
        LANE_X_POSITION = 0
        return
    while _friendIsMoving():
        sleep(0.5)
    # calculate 
    bestDist = 0.0
    numX = 5
    limX = GOAL_WIDTH * 0.5 - 0.2
    x = -limX
    bestX = x
    stepX = 2 * limX / numX
    for iX in range(numX+1):
        x = -limX + iX * stepX
        minDist = 9
        for obst in robot.obstacles():
            minDist = min(minDist, abs(obst[0] - x))
        if minDist > bestDist:
            bestDist = minDist
            bestX = x
    # store result
    LANE_X_POSITION = bestX
    _log('choosing lane at x = %6.2f' % (bestX))

def _waitUntilPassReceiverReady():
    # TODO also use distance? probably not robust enough...
    _log("waiting until pass receiver ready ...")
    while _friendIsMoving():
        sleep(0.5)
    _log("pass receiver is ready")

def _calcSimpleGoalPos():
    # TODO avoid any obstacle nearby goal? NO, choose starting orientation such that penalty spot can be used
    return (0, PENALTY_Y, FORWARD_PHI)

def _calcGoalPostPos():
    # choose goalpost with no obstacle nearby
    posX = 0.5 * GOAL_WIDTH + 0.5 * GOAL_POST_WIDTH
    posLeft = Vec2d(-posX, FIELD_SIZE_Y)
    posRight = Vec2d(posX, FIELD_SIZE_Y)
    chooseLeft = True
    try:
        obstLeft = robot.findClosestObstacle(posLeft)
        distLeft = (obstLeft - posLeft).size()
        obstRight = robot.findClosestObstacle(posRight)
        distRight = (obstLeft - posRight).size()
        if distRight < distLeft:
            chooseLeft = False
    except:
        pass
    result = posRight
    if chooseLeft:
        result = posLeft
    return (result.x, result.y, FORWARD_PHI)

def _interceptWrapper(x, y, phi=0, radius=INTERCEPT_RADIUS_SMALL, repositionRadius=INTERCEPT_RADIUS_SMALL):
    # ideally we use teamplay stuff, rather than restoring old python interceptBall
    _log("receiving pass ...")
    robot.behavior("RECEIVE_PASS")
    sleep(1)
    # TODO: avoid that robot gets the ball if it is closer to the other robot
    if not robot.hasBall():
        _log("intercept fallback getBall")
        robot.getBall()
    """
    # robust intercept wrapper
    # postcondition: robot has the ball
    # large intercept radius already helps a lot due to the fallback inside that function,
    # but we should be careful that other robot does not decide to go into fallback and interfere!
    zoneInterceptBall(x, y, phi, radius, repositionRadius)
    # fallback getBall (TODO also searchball? no, anticipate multiCam + it is unlikely that we let the ball slip away beyond vision)
    """

def _passToTeamMember():
    sleep(SLEEP_SETTLE)
    # use motionPlanning pass action to first teammember, not teamplay
    friendPos = robot.getPosition(robot.teamMembers()[0])
    _log("passing to (%6.2f, %6.2f)" % (friendPos.x, friendPos.y))
    robot.passTo(friendPos.x, friendPos.y)
    sleep(SLEEP_AFTER_KICK)

def _receivePass(radius=INTERCEPT_RADIUS_SMALL):
    # use zoneInterceptBall on current position
    pos = robot.ownPosition()
    _interceptWrapper(pos.x, pos.y, pos.Rz, radius=radius)
    # it exits only when robot has ball

def _safeMove(*target):
    # We might need this wrapper because of the following things:
    #  1. when robot has fetched the ball, but there is an obstacle closeby, then tokyoDrift rotation will likely cause a collision
    #     so we should first drive away from obstacle using robotspeed. 
    #     I consider this a limitation of pathPlanning, so in future it should be implemented there.
    #  2. if there is an obstacle at the target position, the robot will either never arrive (because pathPlanning obstacle avoidance will dance around it) 
    #     or push the obstacle aside, causing a collision.
    #  3. it might be the case that there is an obstacle between target position and ball, causing the robot to not see the ball anymore. 
    #     We do not want to explicitly search ball. 
    # 
    # Possible solution: use a new test-specific heightmap and corresponding action. Furthermore, start with a small drive-backwards fallback.

    #
    # TODO - see version in techChallenge branch -- can we avoid all that complexity by improving tp/pp?
    robot.move(*target)
    # TODO: behavior POSITION_TO_FREE_SPOT?


###### role and trick distribution ######

def TC18_run1():
    # initialize
    _waitUntilBothActive()
    trickSequence = _determineTrickSequence() # might skip parts of sequence based on starting orientation
    # TODO: disable goal area obstacle avoidance - we want to be able to get a ball from goal
    # TODO tuning: reduce rotation speed&acc with ball to prevent losing it while turning (ballHandlers are currently our weak point)
    # distribute roles based on proximity to center, assume one starts within 3m from it and the other further away
    if robot.robotCloseBy(0,0,xyTol=3):
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
    robot.getBall()

def _role1_trick1():
    targetPos = _calcPassMainPos()
    _safeMove(*targetPos)
    _waitUntilPassReceiverReady() # sync point
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
        _log('performing skill: simple goal (%d/%d)' % (1+it, NUM_SIMPLE_GOALS))
        _safeMove(*targetPos)
        robot.kick(SIMPLE_GOAL_POWER)
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
    subTargetPos[1] = subTargetPos[1] - 0.30 # reduce y as bit
    # iterate
    for it in range(NUM_GOAL_POST_HITS):
        _log('performing skill: hit goal post (%d/%d)' % (1+it, NUM_GOAL_POST_HITS))
        # no safe move, because there should not be obstacles near targetPos
        robot.move(*subTargetPos) # coarse move
        sleep(0.5) # settle
        robot.move(*targetPos, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI) # increase accuracy by moving a bit forward
        # abuse a PASS to kick softly after precise aiming ...
        # (actually, we now seem to require a better test interface: to shootAt target with preset kick power)
        passTarget = list(goalPostPos)
        passTarget[1] += 1.0 # project target a bit further to make the ball roll a bit faster, to reduce sensitivity due to field curvature and ball imperfections
        robot.passTo(passTarget[0], passTarget[1])
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
        _log('performing skill: hit goal bar (%d/%d)' % (1+it, NUM_GOAL_BAR_HITS))
        robot.move(*subTargetPos) # coarse move
        sleep(0.5) # settle
        robot.move(*targetPos, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI) # increase accuracy by moving a bit forward
        robot.kick(GOAL_BAR_KICK_POWER, GOAL_BAR_KICK_HEIGHT)  # TODO: tuned on robot 4, verify before tech challenge!
        robot.setVelocity(1, 0, 0, 1) # move 1 meter to the right to avoid the bouncing ball; assume no obstacles nearby (that's why we choose this position)move 1 meter to the right
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

