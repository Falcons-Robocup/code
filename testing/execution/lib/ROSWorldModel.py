#!/usr/bin/env python
import roslib; 
from wx._misc import Sleep
roslib.load_manifest('worldModel') 
import rospy

from math import radians
from math import degrees

from worldModel.srv import *
from worldModel.WorldModelNames.WorldModelInterfaces import WorldModelInterfaces

from robot.api import logger

class ROSWorldModel(object):
    '''
    Class to interact with the WorldModel
    '''
    def __init__(self):
        pass
        
    def set_own_robot_position(self, robotPos):
        rospy.wait_for_service(WorldModelInterfaces.S_SET_OWN_LOCATION)
        try:
            assert(len(robotPos) == 3)
            loc_req = set_own_locationRequest()
            loc_req.confidence = float(0.1)
            loc_req.ownRobotPos.x = float(robotPos[0])
            loc_req.ownRobotPos.y = float(robotPos[1])
            loc_req.ownRobotPos.theta = radians(float(robotPos[2]))
            
            set_own_loc = rospy.ServiceProxy(WorldModelInterfaces.S_SET_OWN_LOCATION, set_own_location)
            _ = set_own_loc(loc_req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
            
    def set_team_robot_position(self, robotNr, robotPos):
        rospy.wait_for_service(WorldModelInterfaces.S_SET_MEMBER_LOCATION)
        try:
            assert(len(robotPos) == 3)
            loc_req = set_member_locationRequest()
            loc_req.robotPos.confidence = float(0.1)
            loc_req.robotID.data = int(robotNr)
            loc_req.robotPos.Pos.x = float(robotPos[0])
            loc_req.robotPos.Pos.y = float(robotPos[1])
            loc_req.robotPos.Pos.theta = radians(float(robotPos[2]))
            loc_req.robotPos.Vel.x = 0.0
            loc_req.robotPos.Vel.y = 0.0
            loc_req.robotPos.Vel.theta = 0.0
            
            set_member_loc = rospy.ServiceProxy(WorldModelInterfaces.S_SET_MEMBER_LOCATION, set_member_location)
            _ = set_member_loc(loc_req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
                    
    
    def set_own_ball_relative_position(self, ballPos):
        rospy.wait_for_service(WorldModelInterfaces.S_SET_OWN_BALL_LOCATION)
        try:
            assert(len(ballPos) == 3)
            loc_req = set_own_ball_locationRequest()
            loc_req.confidence = float(0.1)
            loc_req.angle = radians(float(ballPos[0]))
            loc_req.radius = float(ballPos[1])
            loc_req.height = float(ballPos[2])
            
            set_own_ball_loc = rospy.ServiceProxy(WorldModelInterfaces.S_SET_OWN_BALL_LOCATION, set_own_ball_location)
            _ = set_own_ball_loc(loc_req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    
    def set_own_front_camera_ball_relative_position(self, ballPos):
        rospy.wait_for_service(WorldModelInterfaces.S_SET_OWN_FRONT_CAMERA_BALL_LOCATION)        
        try:
            assert(len(ballPos) == 5)
            loc_req = set_own_front_camera_ball_locationRequest()
            
            loc_req.angle = radians(float(ballPos[0]))
            loc_req.radius = float(ballPos[1])
            loc_req.distance = float(ballPos[2])
            loc_req.cameraHeight = float(ballPos[3])
            loc_req.cameraFrontOffset = float(ballPos[4])
            
            set_own_front_cam_ball_loc = rospy.ServiceProxy(WorldModelInterfaces.S_SET_OWN_FRONT_CAMERA_BALL_LOCATION, set_own_front_camera_ball_location)
            _ = set_own_front_cam_ball_loc(loc_req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
      
    def get_own_robot_location(self):
        rospy.wait_for_service(WorldModelInterfaces.S_GET_OWN_LOCATION)
        try:
            response = list()
            loc_req = get_own_locationRequest()
            
            get_own_loc = rospy.ServiceProxy(WorldModelInterfaces.S_GET_OWN_LOCATION, get_own_location)
            get_own_loc_response = get_own_loc(loc_req)
            response.append(get_own_loc_response.ownRobot.Pos.x)
            response.append(get_own_loc_response.ownRobot.Pos.y)
            response.append(degrees(get_own_loc_response.ownRobot.Pos.theta))
            
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def get_team_robot_location(self, robotNr):
        rospy.wait_for_service(WorldModelInterfaces.S_GET_TEAMMEMBERS)
        try:
            response = list()
            loc_req = get_teammembersRequest()
            
            get_member_loc = rospy.ServiceProxy(WorldModelInterfaces.S_GET_TEAMMEMBERS, get_teammembers)
            get_teammembers_response = get_member_loc(loc_req)
            
            robotFound = False            
            for member in get_teammembers_response.teammembers.RobotArray:
                if member.robotID == int(robotNr):
                    robotFound = True
                    response.append(member.Pos.x)
                    response.append(member.Pos.y)
                    response.append(degrees(member.Pos.theta))
            
            assert(robotFound == True)            
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
                        
    def get_own_robot_velocity(self):
        rospy.wait_for_service(WorldModelInterfaces.S_GET_OWN_LOCATION)
        try:
            response = list()
            loc_req = get_own_locationRequest()
            
            get_own_loc = rospy.ServiceProxy(WorldModelInterfaces.S_GET_OWN_LOCATION, get_own_location)
            get_own_loc_response = get_own_loc(loc_req)
            response.append(get_own_loc_response.ownRobot.Vel.x)
            response.append(get_own_loc_response.ownRobot.Vel.y)
            response.append(degrees(get_own_loc_response.ownRobot.Vel.theta))
            
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
            
    def get_team_robot_velocity(self, robotNr):
        rospy.wait_for_service(WorldModelInterfaces.S_GET_TEAMMEMBERS)
        try:
            response = list()
            loc_req = get_teammembersRequest()
            
            get_member_loc = rospy.ServiceProxy(WorldModelInterfaces.S_GET_TEAMMEMBERS, get_teammembers)
            get_teammembers_response = get_member_loc(loc_req)
            
            robotFound = False            
            for member in get_teammembers_response.teammembers.RobotArray:
                if member.robotID == int(robotNr):
                    robotFound = True
                    response.append(member.Vel.x)
                    response.append(member.Vel.y)
                    response.append(degrees(member.Vel.theta))
            
            assert(robotFound == True)            
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def get_team_robots_IDs(self):
        rospy.wait_for_service(WorldModelInterfaces.S_GET_TEAMMEMBERS)
        try:
            response = list()
            loc_req = get_teammembersRequest()
            
            get_member_loc = rospy.ServiceProxy(WorldModelInterfaces.S_GET_TEAMMEMBERS, get_teammembers)
            get_teammembers_response = get_member_loc(loc_req)
                        
            for member in get_teammembers_response.teammembers.RobotArray:
                response.append(member.robotID)
                       
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
            
    def get_all_team_members(self):
        rospy.wait_for_service(WorldModelInterfaces.S_GET_TEAMMEMBERS)
        try:
            response = list()
            loc_req = get_teammembersRequest()
            
            get_member_loc = rospy.ServiceProxy(WorldModelInterfaces.S_GET_TEAMMEMBERS, get_teammembers)
            get_teammembers_response = get_member_loc(loc_req)
            
            for member in get_teammembers_response.teammembers.RobotArray:
                response.append(member)
                
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
            
    
    def get_absolute_ball_position(self):
        rospy.wait_for_service(WorldModelInterfaces.S_GET_BALL_LOCATION)
        try:
            response = list()
            loc_req = get_ball_locationRequest()
            get_ball_loc = rospy.ServiceProxy(WorldModelInterfaces.S_GET_BALL_LOCATION, get_ball_location)
            ball_loc_response = get_ball_loc(loc_req)                            
            response.append(ball_loc_response.ballPos.BallArray[0].Pos.x)
            response.append(ball_loc_response.ballPos.BallArray[0].Pos.y)
            response.append(ball_loc_response.ballPos.BallArray[0].Pos.z)
                     
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def force_worldModel_recalculation(self):
        rospy.wait_for_service(WorldModelInterfaces.S_FORCE_RECALCULATION)
        try:
            loc_req = force_recalculationRequest()
            force_recalc = rospy.ServiceProxy(WorldModelInterfaces.S_FORCE_RECALCULATION, force_recalculation)
            _ = force_recalc(loc_req)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            