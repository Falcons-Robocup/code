#!/usr/bin/env python
#
# JFEI 2016-05-03 
# Stimulate the robot_pose_ekf node with our own capture data.
# EKF = Extended Kalman Filter.
# A capture is a timed sequence of worldModel inputs. Example:
#
# This script starts the EKF node and feeds it our data.
# Then we analyze / tune.

 
import sys, os
import roslib
import rospy
from threading import Thread
roslib.load_manifest('robot_pose_ekf')
roslib.load_manifest('common')
from FalconsTrace import trace
from time import sleep
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import datetime
import tf



# settings
L = 9999
cov_x = 100.0
cov_y = 100.0
cov_phi = L # 0.1
WITH_PHI = False
WITH_ENC = False
LARGE_COV = [L, 0, 0, 0, 0, 0,
             0, L, 0, 0, 0, 0,
             0, 0, L, 0, 0, 0,
             0, 0, 0, L, 0, 0,
             0, 0, 0, 0, L, 0,
             0, 0, 0, 0, 0, L]
STOP = False
FRAME_ID = "fcs"


def startEKF():
    cmd = "roslaunch robot_pose_ekf_1.launch"
    trace("running command: " + cmd)
    os.system(cmd)
    trace("command finished")


    
def feedCaptureEKF(playbackfile):
    # setup topic publishers    
    topicSpeed  = rospy.Publisher('odom', Odometry, queue_size = 5)
    topicVision = rospy.Publisher('vo', Odometry, queue_size = 5)
    # playback the file
    t0 = datetime.datetime.now()
    for line in file(playbackfile).readlines():
        if STOP: # stop signal from run()
            break
        # split line
        words = line.split()
        if len(words) < 2:
            break # done
        # construct the output message
        # note that we do this before sleeping, to minimize the timing error
        msg = Odometry()
        msg.header.frame_id = FRAME_ID
        msg.child_frame_id = FRAME_ID
        phi = 0.0
        if words[1] == "encoder":
            if not WITH_ENC:
                continue
            outputTopic = topicSpeed
            msg.twist.twist.linear.x = float(words[2])
            msg.twist.twist.linear.y = float(words[3])
            msg.twist.twist.angular.z = float(words[4])
            msg.twist.covariance = [cov_x, 0, 0, 0, 0, 0,   # covariance on x position
                                    0, cov_y, 0, 0, 0, 0,   # covariance on y position
                                    0, 0, L, 0, 0, 0,       # we don't use 3rd dimension, so don't care
                                    0, 0, 0, L, 0, 0,       #
                                    0, 0, 0, 0, L, 0,       #
                                    0, 0, 0, 0, 0, cov_phi] # covariance on orientation
            # set pose covariance to something large for no effect
            msg.pose.covariance = LARGE_COV
        else: # vision
            outputTopic = topicVision
            msg.pose.pose.position.x = float(words[2])
            msg.pose.pose.position.y = float(words[3])
            if WITH_PHI:
                phi = float(words[4])
            msg.twist.covariance = LARGE_COV
            msg.pose.covariance = [cov_x, 0, 0, 0, 0, 0,   # covariance on x position
                                   0, cov_y, 0, 0, 0, 0,   # covariance on y position
                                   0, 0, L, 0, 0, 0,       # we don't use 3rd dimension, so don't care
                                   0, 0, 0, L, 0, 0,       #
                                   0, 0, 0, 0, L, 0,       #
                                   0, 0, 0, 0, 0, cov_phi] # covariance on orientation
        # construct a valid quaternion from yaw (createQuaternionMsgFromYaw)
        # answers.ros.org/question/12903/quaternions-with-python/
        q = tf.transformations.quaternion_from_euler(0, 0, phi)
        msg.pose.pose.orientation = Quaternion(*q)
        # sleep if needed
        elapsed = datetime.datetime.now() - t0
        sleeptime = float(words[0]) - elapsed.total_seconds()
        if sleeptime > 0:
            sleep(sleeptime)
        # publish
        current_time = rospy.Time.now()
        msg.header.stamp = current_time
        outputTopic.publish(msg)
        trace(line)



class cRobotPoseEkfInterface:
    
    def __init__(self, playbackfile):
        trace("initializing")
        # setup the threads
        self.threads = []
        self.t0 = datetime.datetime.now()
        self.threads.append(Thread(target = startEKF))
        self.threads.append(Thread(target = feedCaptureEKF, args=(playbackfile,)))
        # subscribe to EKF output topic
        rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.cbOdomCombined)
        trace("initialization done")

    def cbOdomCombined(self, data):
        trace(str(data))
        elapsed = datetime.datetime.now() - self.t0
        print "%10.6f   kalman  %10.6f  %10.6f  %10.6f" % (elapsed.total_seconds(), data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.z)

    def run(self):
        self.threads[0].start()
        sleep(3)
        rospy.init_node('captureToRosEKF')
        self.threads[1].start()
        rospy.spin()
        # shutdown
        trace('shutdown')
        global STOP
        STOP = True
        for t in self.threads:
            t.join()

if __name__ == '__main__':  
    
    c = cRobotPoseEkfInterface(sys.argv[1])

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

