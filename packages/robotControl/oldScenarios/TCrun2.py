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

# tuned at Locht on 2017-07-03 
# r2 as supporter (role2) and r4 as main (role1)

# settings
# S prefix: which 'skill' (see rules)
S0_POS_1   = ( 0.00,  3.00,  0.50*pi) # face initial ball
S0_POS_2   = ( 0.00, -1.00,  0.50*pi) # wait position, in sight range of S0_POS_1
S3_POS_1   = ( 0.00,  6.00,  0.50*pi) # goal attempt position
S1_POS_1   = ( 0.00,  3.00, -0.50*pi) # face other robot
S1_POS_2   = ( 0.00, -3.00,  0.50*pi) # face other robot
S2_POS_1_C = ( 0.00,  6.00, -0.50*pi) # lob pass attempt position - coarse
S2_POS_1_F = ( 0.00,  5.00, -0.50*pi) # lob pass attempt position - fine
S2_POS_2   = ( 0.00, -6.00,  0.50*pi) # face other robot
S5_POS_1_C = ( 0.97,  6.00,  0.50*pi) # goal post attempt position - coarse
S5_POS_1_F = ( 0.97,  6.30,  0.50*pi) # goal post attempt position - fine
S5_POS_2   = (-4.00,  4.00,  0.25*pi) # goal post support intercept
S4_POS_1_C = ( 0.00,  6.00,  0.50*pi) # goal bar attempt position - coarse
S4_POS_1_F = ( 0.00,  6.30,  0.50*pi) # goal bar attempt position - fine
S4_POS_1_R = ( 0.00,  2.00,  0.50*pi) # goal bar attempt position - receive (further away)

INTERCEPT_RADIUS_SMALL    =  3.0
INTERCEPT_RADIUS_LARGE    =  6.0
TOLERANCE_FINE_XY         =  0.01
TOLERANCE_FINE_PHI        =  0.005
SLEEP_AFTER_PASS          =  2.0
S3_NUM_SIMPLE_GOALS       =  4
S3_SIMPLE_GOAL_POWER      = 50.0
S1_NUM_LOW_PASSES_PAIRS   =  1 # one for each robot
S2_NUM_HIGH_PASSES        =  2 # each high pass sent from robot with role1 is followed by a low pass back from the other 
S2_LOB_PASS_KICK_POWER    = 86.0
S2_LOB_PASS_KICK_HEIGHT   =200
S2_LOB_RECEIVE_SLEEP      =  5.0
S5_NUM_GOAL_POST_HITS     =  3
S5_GOAL_POST_HIT_POWER    = 45.0
S4_NUM_GOAL_BAR_ATTEMPTS  =  4
S4_GOAL_BAR_KICK_POWER    = 78.0
S4_GOAL_BAR_KICK_HEIGHT   =200
S4_SPRINT_BACK_SPEED      =  2.0
S4_SPRINT_BACK_DISTANCE   =  8.0



def TCrun2_interceptWrapper(x, y, phi, radius=INTERCEPT_RADIUS_SMALL, repositionRadius=INTERCEPT_RADIUS_SMALL):
    # robust intercept wrapper
    # postcondition: robot has the ball
    # large intercept radius already helps a lot due to the fallback inside that function, 
    # but we should be careful that other robot does not decide to go into fallback and interfere!
    zoneInterceptBall(x, y, phi, radius, repositionRadius) 
    # TODO fallback getBall + searchball?
    # since we play on large field, and we have to deal with a bouncing ball, 
    # we may need a layer of robustness in case the ball escapes our vision?
    sleep(1)
    if not hasBall():
        log('WARNING: we should have the ball now!!!', 0)
        # for now let's assume the risk of zoneInterceptBall failure above is small
        getBall() # this could cause robot to harass other robot
    
def passTo(*target):
    sleep(1) # settle
    actionBlocking("shoot shootType=passTowardsNearestTeammember") # using teamplay to pass
    # TODO this only works for two robots inplay .... I'd rather explicitly pass to target
    sleep(SLEEP_AFTER_PASS)

def lobPassTo(*target):
    # ignore target, perform a static kick, aiming for bounce repro
    # to maximize pass success chance
    sleep(1) # settle
    safeKick(S2_LOB_PASS_KICK_POWER, S2_LOB_PASS_KICK_HEIGHT)
    sleep(SLEEP_AFTER_PASS)

def TCrun2_waitLobReceive(*waitBallPos):
    return # disabled, use standard intercept + long distance, rules seem to allow for this
    while True:
        if ballCloseBy(waitBallPos[0], waitBallPos[1]):
            break
        sleep(0.05)
    sleep(S2_LOB_RECEIVE_SLEEP)
        
def TCrun2_waitReadyForPasses(*pos):
    while not ballCloseBy(pos[0], pos[1]):
        sleep(0.2)

# end of helper functions
# below main role sequences

def TCrun2_role1():
    """
    Main execution role. Choose a robot with good lob shot reproducability.
    """
    # initialize
    move(*S0_POS_1)
    getBall()
    
    # TODO: disable goal area obstacle avoidance
    # we want to be able to get a ball from goal
    
    # skill 3: shoot on goal a few times and receive the ball
    log('performing skill3 (simple goal) %d times' % S3_NUM_SIMPLE_GOALS, 0)
    for it in range(S3_NUM_SIMPLE_GOALS):
        move(*S3_POS_1, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI)
        safeKick(S3_SIMPLE_GOAL_POWER)
        sleep(SLEEP_AFTER_PASS)
        TCrun2_interceptWrapper(*S3_POS_1)
        
    # move to pass position, signaling other robot to wake up
    move(*S1_POS_1)
    log('performing skill1 (low pass)', 0)
    sleep(1)
    
    # skill 1: low passes
    for it in range(S1_NUM_LOW_PASSES_PAIRS):
        move(*S1_POS_1)
        # standard low pass to robot with role2
        passTo(*S1_POS_2)
        # receive standard low pass from robot with role2, with getball fallback
        TCrun2_interceptWrapper(*S1_POS_1)
        
    # skill 5: hit the goal post
    # aim slightly at the interior, so if we hit the side, other robot can help catching the ball
    log('performing skill5 (goal post hit) %d times' % S5_NUM_GOAL_POST_HITS, 0)
    for it in range(S5_NUM_GOAL_POST_HITS):
        move(*S5_POS_1_C, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI) # coarse move
        sleep(1) # settle
        move(*S5_POS_1_F, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI) # increase accuracy by moving a bit forward
        sleep(1) # settle
        safeKick(S5_GOAL_POST_HIT_POWER)
        sleep(SLEEP_AFTER_PASS)
        # use a very large radius to make sure robot will follow the ball (intercept-internal fallback)
        # even if support robot helps
        TCrun2_interceptWrapper(*S5_POS_1_C, radius=INTERCEPT_RADIUS_LARGE)
        
    # skill 4: hit the goal bar
    log('performing skill4 (goal bar hit) %d times' % S4_NUM_GOAL_BAR_ATTEMPTS, 0)
    for it in range(S4_NUM_GOAL_BAR_ATTEMPTS):
        move(*S4_POS_1_C, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI) # coarse move
        sleep(1) # settle
        move(*S4_POS_1_F, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI) # increase accuracy by moving a bit forward
        sleep(1) # settle
        safeKick(S4_GOAL_BAR_KICK_POWER, S4_GOAL_BAR_KICK_HEIGHT)
        # immediately back away to improve success rate of intercepting the ball
        speed(0, -S4_SPRINT_BACK_SPEED, 0, abs(S4_SPRINT_BACK_DISTANCE / S4_SPRINT_BACK_SPEED))
        TCrun2_interceptWrapper(*S4_POS_1_R, radius=INTERCEPT_RADIUS_LARGE, repositionRadius=INTERCEPT_RADIUS_LARGE)

    # skill 2: lob pass - hardest one last
    # tune lob settings such that there is 1 or 2 intermediate bounce(s), this increases chance for good receive?
    # rules do not specify number of bounces nor distance, so we could simply choose maximum distance, 
    # probably ball will stop bouncing so support robot can do regular intercept
    log('performing skill2 (high pass)', 0)
    for it in range(S2_NUM_HIGH_PASSES):
        move(*S2_POS_1_C, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI)
        move(*S2_POS_1_F, xyTol=TOLERANCE_FINE_XY, phiTol=TOLERANCE_FINE_PHI)
        sleep(1)
        lobPassTo(*S2_POS_2)
        TCrun2_interceptWrapper(*S2_POS_1_C, radius=INTERCEPT_RADIUS_LARGE)
        
    log('done', 0)
    
    
def TCrun2_role2():
    """
    Support role. Choose a robot with strong ballhandlers to receive lob shot.
    """

    # initialize
    move(*S1_POS_2)
    
    # while robot with role1 is doing other stuff, we go into the first wait
    TCrun2_waitReadyForPasses(*S1_POS_1)
    move(*S1_POS_2)

    # skill 1: low pass
    log('performing skill1 (low pass)', 0)
    for it in range(S1_NUM_LOW_PASSES_PAIRS):
        # standard low pass 
        TCrun2_interceptWrapper(*S1_POS_2)
        move(*S1_POS_2)
        # send ball back
        passTo(*S1_POS_1)
        
    # skill 2: lob pass
    # instead of counting, do this indefinitely, to also be robust for rogue balls
    log('performing skill2 (high pass) - support', 0)
    #for it in range(S2_NUM_HIGH_PASSES):
    while True:
        # do not do a standard intercept, because it will probably move erratically due to high ball, 
        # instead try to tune lob such that the ball ends up in ballHandlers after one or two bounces
        move(*S2_POS_2)
        TCrun2_waitLobReceive(*S2_POS_1_F)
        TCrun2_interceptWrapper(*S2_POS_2, radius=INTERCEPT_RADIUS_LARGE)
        move(*S2_POS_2)
        # send ball back
        passTo(*S2_POS_1_C)
        
    
