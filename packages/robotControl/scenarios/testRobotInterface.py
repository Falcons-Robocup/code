""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 from robotScenarioBase import *


def testRobotInterface():
    """
    A series of tests to check if all functionality provided by robotInterface works.
    """

    print
    print "TEST: simulation environment"
    print "isSimulated =", robot.isSimulated()
    print

    print "TEST: shutdown flag"
    print "isShutDown =", robot.isShutDown()
    assert(robot.isShutDown() == False)
    print

    print "TEST: own robot ID"
    print robot.myRobotId()
    print

    print "TEST: stop (nothing should happen)"
    robot.stop()
    print

    print "TEST: enable ballHandlers"
    robot.enableBallHandlers()
    sleep(1)
    print

    print "TEST: disable ballHandlers"
    robot.disableBallHandlers()
    sleep(1)
    print

    print "TEST: velocity setpoint (half rotation)"
    robot.setVelocity(0, 0, 1, 3.14)
    print

    print "TEST: get current position"
    pos = robot.getPosition()
    print "position =", pos
    print

    print "TEST: get current velocity"
    vel = robot.getVelocity()
    print "velocity =", vel
    print

    print "TEST: move a bit"
    robot.move(pos.x, pos.y + 0.5, pos.Rz)
    print

    print "TEST: which robots are active"
    print robot.activeRobots()
    print

    print "TEST: teammembers are all robots except self"
    teamMembers = robot.teamMembers()
    print teamMembers
    assert(robot.myRobotId() not in teamMembers)
    print

    print "TEST: relative index"
    print robot.myRelIndex()
    print

    print "TEST: ball possession as enum"
    print robot.ballPossession()
    print

    print "TEST: ball possession as boolean"
    print robot.hasBall()
    print

    print "TEST: does team see a ball"
    print robot.seeBall()
    print

    print "TEST: closest obstacle"
    print robot.findClosestObstacle(1, 6)
    print

    print "TEST: robot close to penalty spot"
    print (robot.robotCloseBy(0, 6, 2.0) or robot.robotCloseBy(0, -6, 2.0))
    print

    if robot.seeBall():
        print "TEST: ball position"
        print robot.ballPosition()
        print

        print "TEST: ball velocity"
        print robot.ballVelocity()
        print

        print "TEST: ball on same half"
        print robot.ballOnSameHalf()
        print

        print "TEST: ball close to penalty spot"
        print (robot.ballCloseBy(0, 6, 2.0) or robot.ballCloseBy(0, -6, 2.0))
        print

        print "TEST: get ball"
        robot.getBall()
        sleep(0.1) # give async wm thread a bit of time ...
        assert(robot.hasBall())
        print




