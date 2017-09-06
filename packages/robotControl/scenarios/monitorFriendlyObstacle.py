""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # load basic commands
from robotRosInterface import *
import numpy



def monitorFriendlyObstacle(timeout=10, robotIds=None):
    """
    Monitor function for obstacle tracking.
    Report the average obstacle position error and its standard deviation.
    Requires friendly robot to be active.
    """
    dt = 0.05 # 20Hz, which is slower than heartbeat (30Hz), to make sure each sample is unique
    timeout = float(timeout)
    # disable friendly obstacle filtering
    setConfig("worldModelNode/obstacleTracker", {"filterXYmemberTolerance": 0.0})
    # determine active robots
    if robotIds == None:
        robotIds = activeRobots()
    else:
        robotIds = [int(s) for s in str(robotIds).split()]
    # fill the lists with measurements
    ex = []
    ey = []
    evx = []
    evy = []
    ev = []
    elapsed = 0
    tStart = time()
    while elapsed < timeout:
        sleep(dt)
        # for each active other robot:
        for r in robotIds:
            if r != myRobotId():
                # find closest obstacle
                if 1:
                    robotPos = getPosition(r)
                    robotVel = getVelocity(r)
                    (obstPos, obstVel) = findClosestObstacle(robotPos)
                    log("rx=%6.2f ry=%6.2f:" % (robotPos[0], robotPos[1]), 1)
                    log("ox=%6.2f oy=%6.2f:" % (obstPos[0], obstPos[1]), 1)
                    evx.append(abs(obstVel[0] - robotVel[0]))
                    evy.append(abs(obstVel[1] - robotVel[1]))
                    obstSpeed = sqrt(obstVel[0]**2 + obstVel[1]**2)
                    robotSpeed = sqrt(robotVel[0]**2 + robotVel[1]**2)
                    ev.append(abs(obstSpeed - robotSpeed))
                    ex.append(abs(obstPos[0] - robotPos[0]))
                    ey.append(abs(obstPos[1] - robotPos[1]))
                #except:
                #    # apparently no obstacle found ...?!
                #    pass 
        elapsed = time() - tStart
    # calculate and display statistics
    if len(ex):
        log("obstacle accuracy results after %6.2f seconds (%d samples):" % (elapsed, len(ex)), 0)
        log("average position error  : (%8.4f, %8.4f)" % (numpy.mean(ex), numpy.mean(ey)), 0)
        log("stddev position error   : (%8.4f, %8.4f)" % (numpy.std(ex), numpy.std(ey)), 0)
        log("average velocity error  : (%8.4f, %8.4f)" % (numpy.mean(evx), numpy.mean(evy)), 0)
        log("stddev velocity error   : (%8.4f, %8.4f)" % (numpy.std(evx), numpy.std(evy)), 0)
        log("average speed error     :  %8.4f" % (numpy.mean(ev)), 0)
        log("stddev speed error      :  %8.4f" % (numpy.std(ev)), 0)
    else:
        log("did not get any samples ...", 0)

