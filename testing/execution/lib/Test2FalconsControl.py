#!/usr/bin/env python
import rospy
import sys, os
import time, datetime
import copy
import math
import roslib
import roslib; 
roslib.load_manifest('robotControl')
roslib.load_manifest('analyzer')
sys.path.append("/home/robocup/falcons/code/packages/robotControl")
import FalconsMath
from simControl import *
from analyzerPostMatch import AnalyzerPostMatch



# large tolerance for now (we have simulated vision noise, slight movement 
# at initialization before pathplanning is disabled, numeric randomness
XYTOL = 0.5
PHITOL = 0.2


class Test2FalconsControl(object):
    '''
    Class to interact with the control layer
    '''
    def __init__(self):
        self.logging = None
        
    def execute_control(self, s):
        s = str(s)
        print "execute_control", s
        if s.startswith("activate"):
            w = s.split()
            s = "activate " + w[1][4] + w[2]
        simCommand(s)

    def initialize_simulator(self, myteam = "teamA"):
        print "initialize_simulator"
        simInit()
        time.sleep(3)

    def shutdown_simulator(self):
        print "shutdown_simulator"
        simStop()
        time.sleep(3)

    def load_logging(self):
        self.logging = AnalyzerPostMatch()
    
    def check_no_errors(self):
        errors = self.logging.getErrors()
        assert(len(errors) == 0)
    
    def check_no_collisions(self):
        collisions = self.logging.getCollisions()
        assert(len(collisions) == 0)
    
    def verify_ball_possession(self, num_seconds, robot_number, team_name):
        # check logging is loaded
        if (self.logging == None):
            raise RuntimeError("logging has not been loaded!")
        # sanitize inputs
        num_seconds = float(num_seconds)
        robot_number = int(robot_number)
        team_name = str(team_name)
        assert(team_name in ["teamA", "teamB"])
        assert(robot_number in range(6))
        # get the data
        msg = self.logging.getMessage("/teamA/g_worldmodel", num_seconds)
        # perform checks
        if team_name == "teamA":
            assert(msg.ballPossession.type == 3)
            assert(msg.ballPossession.robotID == robot_number)
        if team_name == "teamB":
            assert(msg.ballPossession.type == 2)
            assert(msg.ballPossession.robotID == 0)
            
    def verify_ball_possession_field(self, num_seconds):        
        # check logging is loaded
        if (self.logging == None):
            raise RuntimeError("logging has not been loaded!")
        # sanitize inputs
        num_seconds = float(num_seconds)
        # get the data
        msg = self.logging.getMessage("/teamA/g_worldmodel", num_seconds)
        # perform checks
        assert(msg.ballPossession.type == 1)
                    
    def verify_ball_position(self, num_seconds, position, robot_number = 1):
        # check logging is loaded
        if (self.logging == None):
            raise RuntimeError("logging has not been loaded!")
        # sanitize inputs
        num_seconds = float(num_seconds)
        robot_number = int(robot_number)
        pos_vals = [float(v) for v in position.split()]
        check_x = pos_vals[0]
        check_y = pos_vals[1]
        # get the data
        msg = self.logging.getMessage("/teamA/g_worldmodel", num_seconds)
        # perform checks
        x = msg.ballpos.x
        y = msg.ballpos.y
        dx = abs(x - check_x)
        dy = abs(y - check_y)
        xy_error = math.sqrt(dx*dx + dy*dy)
        print "xy position error  : %6.2f" % (xy_error)
        if (xy_error > XYTOL):
            print msg
            raise Exception("position mismatch")

    def verify_position(self, num_seconds, robot_number, team_name, position, pos_type):
        # check logging is loaded
        if (self.logging == None):
            raise RuntimeError("logging has not been loaded!")
        # sanitize inputs
        assert(pos_type == "worldmodel") # simulated position check not yet supported
        num_seconds = float(num_seconds)
        robot_number = int(robot_number)
        team_name = str(team_name)
        pos_vals = [float(v) for v in position.split()]
        check_x = pos_vals[0]
        check_y = pos_vals[1]
        check_phi = pos_vals[2]
        # get the data
        msg = self.logging.getMessage("/teamA/g_worldmodel", num_seconds)
        # perform checks
        for robot in msg.friends:
            if robot.id == robot_number:
                x = robot.x
                y = robot.y
                dx = abs(x - check_x)
                dy = abs(y - check_y)
                xy_error = math.sqrt(dx*dx + dy*dy)
                phi_error = FalconsMath.phidiff(robot.phi, check_phi)
                print "xy position error  : %6.2f" % (xy_error)
                print "phi position error : %6.2f" % (phi_error)
                if (xy_error > XYTOL) or (phi_error > PHITOL):
                    print msg
                    raise Exception("position mismatch")
                return  
        raise Exception("robot %d not found" % robot_number)

    def set_target(self, robot_number, team_name, target_x, target_y, target_phi):
        # sanitize inputs
        robot_number = int(robot_number)
        team_name = str(team_name)
        target_x = float(target_x)
        target_y = float(target_y)
        target_phi = float(target_phi)
        # give the command
        s = "move %s %d %6.2f %6.2f %6.2f" % (team_name, robot_number, target_x, target_y, target_phi)
        simCommand(s)

    def teleport_ball(self, x, y, vx, vy):
        # sanitize inputs
        x = float(x)
        y = float(y)
        vx = float(vx)
        vy = float(vy)
        # give the command
        s = "teleportball %6.2f %6.2f %6.2f %6.2f" % (x, y, vx, vy)
        simCommand(s)

