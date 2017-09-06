#!/usr/bin/env python
import roslib; 
roslib.load_manifest('heartBeat') 
import rospy

from robot.api import logger

from heartBeat.msg import beat

class ROSHeartBeat(object):
    '''
    Class to mimic the heartBeat node
    '''
    def __init__(self):
        pass
    
    def initialize_heartBeat(self):
        try:
            rospy.init_node('heartBeatNode', anonymous=True)
        except rospy.ServiceException, e:
            logger.error('ros init node failed')
            print "ros init node failed: %s"%e
        
    def generate_ros_heartBeat_signal(self):
        try:
            pub = rospy.Publisher('t_beat', beat, queue_size=1)
            pub.publish()
            
        except rospy.ServiceException, e:
            logger.error('ros topic failed')
            print "ros topic failed: %s"%e