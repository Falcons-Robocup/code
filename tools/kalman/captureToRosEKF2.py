#!/usr/bin/env python
#
# JFEI 2016-05-03 
# Stimulate the robot_localization EKF node with our own capture data.
# EKF = Extended Kalman Filter.
# A capture is a timed sequence of worldModel inputs. Example:
#
# 72.826428  encoder    0.002917   -0.005585    0.258839
# 72.836723  encoder    0.000866    0.008378    0.240350
# 72.846700  vision    -0.074839    0.031975   -2.621974
# 72.846993  encoder   -0.001334   -0.005585    0.249594
#
# This script starts the EKF node and feeds it our data.
# Then we analyze / tune.

 
import sys, os
import argparse
import roslib
import rospy
from threading import Thread
roslib.load_manifest('robot_localization')
roslib.load_manifest('common')
from FalconsTrace import trace
from time import sleep
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped, Quaternion
import datetime
import tf
import math, numpy



# settings
COV_XY_VISION = 0.1
COV_XY_ENC = 0.01
COV_PHI_VISION = 0.2
COV_PHI_ENC = 0.1
L = 9999
WITH_PHI = True
WITH_ENC = True
LARGE_COV = [L, 0, 0, 0, 0, 0,
             0, L, 0, 0, 0, 0,
             0, 0, L, 0, 0, 0,
             0, 0, 0, L, 0, 0,
             0, 0, 0, 0, L, 0,
             0, 0, 0, 0, 0, L]
STOP = False
FRAME_ID = "fcs"



def startEKF():
    cmd = "roslaunch robot_localization_ekf_1.launch &"
    trace("running command: " + cmd)
    os.system(cmd)
    trace("command finished")
        
    
    
def feedCaptureEKF(args):
    global STOP
    # setup topic publishers    
    topicSpeed  = rospy.Publisher('twist', TwistWithCovarianceStamped, queue_size = 5)
    topicVision = rospy.Publisher('pose', PoseWithCovarianceStamped, queue_size = 5)
    # playback the file
    t0 = datetime.datetime.now()
    for line in file(args.inputfile).readlines():
        trace(line.strip())
        if STOP: # stop signal from run()
            break
        # split line
        words = line.split()
        if len(words) < 2:
            break # done
        tcurr = float(words[0])
        # check time window of interest
        if args.preskip and (tcurr < args.tmin):
            continue
        if tcurr > args.tmax:
            STOP = True
            trace("signal shutdown")
            rospy.signal_shutdown("tmax exceeded")
            break
        # construct the output message
        # note that we do this before sleeping, to minimize the timing error
        phi = 0.0
        if words[1] == "encoder":
            msg = TwistWithCovarianceStamped()
            outputTopic = topicSpeed
            if not WITH_ENC:
                continue
            msg.twist.twist.linear.x = float(words[2])
            msg.twist.twist.linear.y = float(words[3])
            msg.twist.twist.angular.z = float(words[4])
            msg.twist.covariance = [COV_XY_ENC, 0, 0, 0, 0, 0,   # covariance on x position
                                    0, COV_XY_ENC, 0, 0, 0, 0,   # covariance on y position
                                    0, 0, L, 0, 0, 0,       # we don't use 3rd dimension, so don't care
                                    0, 0, 0, L, 0, 0,       #
                                    0, 0, 0, 0, L, 0,       #
                                    0, 0, 0, 0, 0, COV_PHI_ENC] # covariance on orientation
            msg.header.frame_id = FRAME_ID
        else: # vision
            msg = PoseWithCovarianceStamped()
            outputTopic = topicVision
            msg.pose.pose.position.x = float(words[2])
            msg.pose.pose.position.y = float(words[3])
            if WITH_PHI:
                phi = float(words[4])
            msg.pose.covariance = [COV_XY_VISION, 0, 0, 0, 0, 0,   # covariance on x position
                                   0, COV_XY_VISION, 0, 0, 0, 0,   # covariance on y position
                                   0, 0, L, 0, 0, 0,       # we don't use 3rd dimension, so don't care
                                   0, 0, 0, L, 0, 0,       #
                                   0, 0, 0, 0, L, 0,       #
                                   0, 0, 0, 0, 0, COV_PHI_VISION] # covariance on orientation
            msg.header.frame_id = FRAME_ID
            # construct a valid quaternion from yaw (createQuaternionMsgFromYaw)
            # answers.ros.org/question/12903/quaternions-with-python/
            q = tf.transformations.quaternion_from_euler(0, 0, phi)
            msg.pose.pose.orientation = Quaternion(*q)
        # sleep if needed
        elapsed = datetime.datetime.now() - t0
        if args.preskip:
            elapsed += datetime.timedelta(seconds=args.tmin)
        sleeptime = tcurr - elapsed.total_seconds()
        if sleeptime > 0:
            sleep(sleeptime)
        # publish
        current_time = rospy.Time.now()
        msg.header.stamp = current_time
        outputTopic.publish(msg)
        trace("published")



class cRobotPoseEkfInterface:
    
    def __init__(self, args):
        trace("initializing")
        # setup the threads
        self.threads = []
        self.t0 = datetime.datetime.now()
        self.args = args
        self.threads.append(Thread(target = startEKF))
        self.threads.append(Thread(target = feedCaptureEKF, args=(args, )))
        # subscribe to EKF output topic
        rospy.Subscriber("odometry/filtered", Odometry, self.cbOdomCombined)
        # data so we can calculate KPI's
        self.kalman_t = []
        self.kalman_x = []
        self.kalman_y = []
        self.kalman_phi = []
        trace("initialization done")

    def cbOdomCombined(self, data):
        trace(str(data))
        elapsed = datetime.datetime.now() - self.t0
        t = elapsed.total_seconds()
        if args.preskip:
            t += args.tmin
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        # ?? magic pi factor
        phi = data.pose.pose.orientation.z * math.pi
        print "%10.6f   kalman  %10.6f  %10.6f  %10.6f" % (t, x, y, phi)
        if (t > self.args.tmin) and (t < self.args.tmax):
            self.kalman_t.append(t)
            self.kalman_x.append(x)
            self.kalman_y.append(y)
            self.kalman_phi.append(phi)

    def run(self):
        self.threads[0].start()
        sleep(3)
        rospy.init_node('captureToRosEKF2')
        self.threads[1].start()
        rospy.spin()
        # shutdown
        trace('shutdown')
        os.system("pkill roslaunch")
        global STOP
        STOP = True
        for t in self.threads:
            t.join()
        trace('joined')
        self.finalize()

    def finalize(self): 
        # parse vision data       
        self.vision_t = []
        self.vision_x = []
        self.vision_y = []
        self.vision_phi = []
        for line in file(self.args.inputfile).readlines():
            # split line
            words = line.split()
            if len(words) < 2:
                break # done
            tcurr = float(words[0])
            # check time window of interest
            if (tcurr > args.tmin) and (tcurr < args.tmax) and (words[1] == 'vision'):
                self.vision_t.append(tcurr)
                self.vision_x.append(float(words[2]))
                self.vision_y.append(float(words[3]))
                self.vision_phi.append(float(words[4]))
        # calculate KPI's
        maxgradient_kalman_x = 0.0
        maxgradient_kalman_y = 0.0
        maxgradient_kalman_phi = 0.0
        maxgradient_vision_x = 0.0
        maxgradient_vision_y = 0.0
        maxgradient_vision_phi = 0.0
        for i in range(1, len(self.kalman_t)):
            dt = self.kalman_t[i] - self.kalman_t[i-1]
            maxgradient_kalman_x = max(maxgradient_kalman_x, abs(self.kalman_x[i] - self.kalman_x[i-1]) / dt)
            maxgradient_kalman_y = max(maxgradient_kalman_y, abs(self.kalman_y[i] - self.kalman_y[i-1]) / dt)
            maxgradient_kalman_phi = max(maxgradient_kalman_phi, abs(self.kalman_phi[i] - self.kalman_phi[i-1]) / dt)
        for i in range(1, len(self.vision_t)):
            dt = self.vision_t[i] - self.vision_t[i-1]
            maxgradient_vision_x = max(maxgradient_vision_x, abs(self.vision_x[i] - self.vision_x[i-1]) / dt)
            maxgradient_vision_y = max(maxgradient_vision_y, abs(self.vision_y[i] - self.vision_y[i-1]) / dt)
            maxgradient_vision_phi = max(maxgradient_vision_phi, abs(self.vision_phi[i] - self.vision_phi[i-1]) / dt)
        # print KPI's
        print "         kalman_x    kalman_y  kalman_phi     vision_x    vision_y  vision_phi"
        f = numpy.min
        print "min    %10.6f  %10.6f  %10.6f   %10.6f  %10.6f  %10.6f" % (f(self.kalman_x), f(self.kalman_y), f(self.kalman_phi), f(self.vision_x), f(self.vision_y), f(self.vision_phi))
        f = numpy.max
        print "max    %10.6f  %10.6f  %10.6f   %10.6f  %10.6f  %10.6f" % (f(self.kalman_x), f(self.kalman_y), f(self.kalman_phi), f(self.vision_x), f(self.vision_y), f(self.vision_phi))
        f = numpy.mean
        print "mean   %10.6f  %10.6f  %10.6f   %10.6f  %10.6f  %10.6f" % (f(self.kalman_x), f(self.kalman_y), f(self.kalman_phi), f(self.vision_x), f(self.vision_y), f(self.vision_phi))
        f = numpy.std
        print "sigma  %10.6f  %10.6f  %10.6f   %10.6f  %10.6f  %10.6f" % (f(self.kalman_x), f(self.kalman_y), f(self.kalman_phi), f(self.vision_x), f(self.vision_y), f(self.vision_phi))
        print "grad   %10.6f  %10.6f  %10.6f   %10.6f  %10.6f  %10.6f" % (maxgradient_kalman_x, maxgradient_kalman_y, maxgradient_kalman_phi, maxgradient_vision_x, maxgradient_vision_y, maxgradient_vision_phi)
        sleep(1)
        sys.exit(0)



if __name__ == '__main__':  
    # Argument parsing.
    parser     = argparse.ArgumentParser(description='extend a capture with encoder values')
    parser.add_argument('--tmin', help='start of time window of interest', default=0.0, type=float)
    parser.add_argument('--tmax', help='end of time window of interest', default=999.0, type=float)
    parser.add_argument('--preskip', help='skip samples before tmin', action='store_true')
    parser.add_argument('inputfile', help='input capture file', type=str, default=None)
    args       = parser.parse_args()
    
    c = cRobotPoseEkfInterface(args)

    c.run()
    


"""

robocup@butter [] falcons_control_20160426_230040 $ rosmsg show nav_msgs/Odometry
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
"""

