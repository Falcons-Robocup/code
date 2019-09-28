""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 from robotScenarioBase import *
from time import time
import threading
import numpy
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt


def _startThread(f, *args):
    t = threading.Thread(target=f, args=args)
    t.start()


# settings
POSITION_1 = (-3, -8, 0)
POSITION_2 = ( 3, -8, 0)
ITERATIONS = 2 
TIMEOUT = 10
SPEED_THRESHOLD = 0.5 # exclude ramp-up ramp-down for accuracy
FREQUENCY = 20 # 20Hz, which is slower than heartbeat (30Hz), to make sure each sample is unique
DUMP_DATA = True
MAKE_PLOTS = False


# calculate function
def _monitorAndCalculateLatency(timeout=TIMEOUT):
    dt = 1.0 / FREQUENCY
    encoderTimes = []
    visionTimes = []
    encoderX = []
    visionX = []
    velocityX = []
    elapsed = 0
    tStart = time()
    measuring = False
    while elapsed < timeout:
        sleep(dt)
        posEncoder = robot.ownPosition()
        vel = robot.ownVelocity()
        if abs(vel.x) < SPEED_THRESHOLD:
            if measuring:
                print "stopping measurement"
                break
        if abs(vel.x) > SPEED_THRESHOLD:
            if measuring == False:
                print "starting measurement"
            measuring = True
        if measuring:
            # process measurement
            encoderTimes.append(time())
            encoderX.append(posEncoder.x)
            vision = robot.bestVisionCandidate()
            if vision != None:
                visionTimes.append(vision[0])
                visionX.append(vision[1].x)
            velocityX.append(vel.x)
        elapsed = time() - tStart
    # make numpy arrays
    encoderTimes = numpy.array(encoderTimes)
    encoderX = numpy.array(encoderX)
    visionTimes = numpy.array(visionTimes)
    visionX = numpy.array(visionX)
    velocityX = numpy.array(velocityX)
    # shift time to something small
    t0 = min(encoderTimes[0], visionTimes[0])
    encoderTimes = encoderTimes - t0
    visionTimes = visionTimes - t0
    # display
    if DUMP_DATA:
        def prettyPrint(n, x):
            print n, "= [", x[0],
            for v in x[1:]:
                print ", %.3f" % (v),
            print "]"
        prettyPrint("visionTimes", visionTimes)
        prettyPrint("visionX", visionX)
        prettyPrint("encoderTimes", encoderTimes)
        prettyPrint("encoderX", encoderX)
        prettyPrint("velocityX", velocityX)
    # calculate latency: we fit both series as a line t(x), evaluate dt at first accepted X sample
    # (so as a side effect we can measure encoder drift)
    fitVision = numpy.polyfit(visionX, visionTimes, 1)
    fitEncoder = numpy.polyfit(encoderX, encoderTimes, 1)
    x1 = visionX[0]
    x2 = visionX[-1]
    # interpolation lines
    if MAKE_PLOTS:
        plt.figure()
        plt.plot(visionTimes, visionX, 'bx')
        plt.plot(encoderTimes, encoderX, 'r+')
        def plotLine(x1, x2, fit):
            line = fit[0] * numpy.array([x1, x2]) + fit[1]
            plt.plot(line, [x1, x2], 'k:')
        plotLine(x1, x2, fitVision)
        plotLine(x1, x2, fitEncoder)
    # report
    latency = numpy.polyval(fitVision - fitEncoder, x1)
    print "latency of vision w.r.t. encoder: %dms" % (int(1e3 * latency))
    print "encoder drift: %.1f%%" % (int(1e2 * (abs(fitEncoder[0]) - fitVision[0])))


def locLatency():
    """
    Measure localization latency (of vision, w.r.t. encoders).
    """
    
    # TODO: limit speed to 1m/s
    # for some reason pathPlanning reconfigure hangs (worldModel runs fine)
    
    for it in range(ITERATIONS):
        robot.restoreConfig("worldModelNode/localization")
        sleep(1)
        robot.move(*POSITION_1)
        sleep(1)
        robot.setConfig("worldModelNode/localization", {"visionOwnWeightFactor": 0.0}) # we will be driving on encoders
        sleep(1)
        _startThread(_monitorAndCalculateLatency, TIMEOUT)
        robot.move(*POSITION_2)
        sleep(1)
        robot.restoreConfig("worldModelNode/localization")
        sleep(1)
        robot.move(*POSITION_2)
        sleep(1)
        robot.setConfig("worldModelNode/localization", {"visionOwnWeightFactor": 0.0}) # we will be driving on encoders
        sleep(1)
        _startThread(_monitorAndCalculateLatency)
        robot.move(*POSITION_1)
        sleep(1)

    if MAKE_PLOTS:
        plt.show()

