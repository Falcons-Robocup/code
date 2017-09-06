""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python

'''
Created on January 25, 2015

@author: Tim Kouters
         Edwin Schreuder 
'''
import roslib; roslib.load_manifest('peripheralsInterface')
import rospy

from simulator.srv import *
from worldModel.srv import *
from peripheralsInterface.srv import *

import datetime
import sys
import traceback
import copy
from math import pi
from numpy.lib.scimath import sqrt, arccos
from FalconsTrace import trace,traceError
      
class cCompass(object):
    
    def __init__(self):
        '''
        Initialize cCompass class
        Steps:
         - Open serial communication
         - Create cyclic timer to read cCompass value
         - Start timer
        '''
        try:                      
            self._angle = 0.0
            self._vel_angle = 0.0
            self._timestamp = datetime.datetime.now()
        except:
            traceError('Failed to initialize Compass')
        
    def _update_angle_speed(self, vel_theta):   
        try:     
            self._calc_angle()
            self._vel_angle = vel_theta
        except:
            traceError('Failed to update angle speed')
        
    def _calc_angle(self):
        'Calculate time difference in seconds'
        try:
            time_now = datetime.datetime.now()
            timediff_sec = (time_now - self._timestamp).microseconds / 1000000.0 
            
            'Calculate new angle with old velocity before updating'
            self._angle += self._vel_angle * timediff_sec
            
            'Update own timestamp'
            self._timestamp = time_now
        except:
            traceError('failed to calculate angle')
        
        
    def _map_0_to_360(self, angle):
        ret_val = angle
        try:
            while (ret_val < 0):
                ret_val += 360.0
            while (ret_val > 360.0):
                ret_val -= 360.0
        except:
            traceError('failed to map angle')
        
        return ret_val
                
    def get_angle(self):
        'read the current _angle from the cCompass.'
        self._calc_angle()
        return self._map_0_to_360(self._angle)
                  
                  
class cMotors(object):
    meter_per_sec_to_encoder_value = 120.0
    rad_per_sec_to_encoder_value = 35.34    
    
    def __init__(self, compassObject):
        '''
        Initialize motors class
        Steps:
         - Open serial ports for driving motors
         - Open serial ports for ball handling motors
        '''
        try:
            self._motor_ports = []
            self._robot_speed_x = 0.0
            self._robot_speed_y = 0.0
            self._robot_speed_theta = 0.0
            self._displacement = {}
            self._vel_motor_left = 0.0
            self._vel_motor_right = 0.0
            self._vel_motor_rear = 0.0
            self._ball_speed_left = 0.0
            self._ball_speed_right = 0.0
            self._timestamp = datetime.datetime.now()
            self._displacementTimestamp = datetime.datetime.now()
            self._compass = compassObject
            self.ball_possession_status = False
            self._bhAngle = 0
            
            self._init_services()
            
            trace('Motors class initialized')
        except:
            traceError('failed to initialize cMotors')
            
    def _init_services(self):
        'Initialize services that peripheralsInterface is providing'
        try:
            self._srv_get_ballHandlers_angle = rospy.Service('s_get_ballHandlers_angle', s_get_ballHandlers_angle, self._getBallHandlersAngle)
            self._srv_set_ballHandlers_angle = rospy.Service('s_set_ballHandlers_angle', s_set_ballHandlers_angle, self._setBallHandlersAngle)
            self._srv_disable_ballHandlers   = rospy.Service('s_disable_ballHandlers',   s_disable_ballHandlers,   self._disableBallHandlers)
            self._srv_enable_ballHandlers    = rospy.Service('s_enable_ballHandlers',    s_enable_ballHandlers,    self._enableBallHandlers)
        except:
            traceError('Failed to init own services')
            print traceback.format_exc()
            print sys.exc_info()[0]
            
    def _setBallHandlersAngle(self, data):
        self._bhAngle = data.angle
        return []
        
    def _getBallHandlersAngle(self, data):
        retVal = s_get_ballHandlers_angleRequest()
        retVal.angle = self._bhAngle
        return retVal
        
    def _disableBallHandlers(self, data):
        return []
        
    def _enableBallHandlers(self, data):
        return []
        
    def _set_speed(self, motor_left_vel, motor_right_vel, motor_rear_vel):
        'set the speed individually to cMotors left,right and rear'      
        
        try:  
            self._vel_motor_left = motor_left_vel
            self._vel_motor_right = motor_right_vel
            self._vel_motor_rear = motor_rear_vel
            
            trace('set speed = %6.2f, %6.2f, %6.2f', motor_left_vel, motor_right_vel, motor_rear_vel)
            
            trace('_set_speed finished')
        except:
            traceError('failed to set_speed')
        
    def _project_angle_mpi_pi(self,angle):   
        
        tmp_angle = copy(angle)
        
        try:
            while tmp_angle < -pi:
                tmp_angle += 2.0*pi
            while tmp_angle > pi:
                tmp_angle -= 2*pi;
        except:
            traceError('failed to project angle')
            
        return tmp_angle
         
        
    def stop_robot(self):
        'stops the robot by sending speed 0'        
        try:
            self._vel_motor_left = 0.0
            self._vel_motor_right = 0.0
            self._vel_motor_rear = 0.0
            
            trace('robot stopped')
        except:
            traceError('Cannot stop robot')
            
    def _read_driving_encoders(self):
        '''
        Read driving encoders and calculate relative position displacements
        '''        
        time_now = datetime.datetime.now()
        timediff_sec = (((time_now - self._displacementTimestamp).microseconds / 1000000.0) + (time_now - self._displacementTimestamp).seconds)
        self._displacementTimestamp = time_now
        
        dx = self._robot_speed_x * timediff_sec  
        dy = self._robot_speed_y * timediff_sec
        dtheta = self._robot_speed_theta * timediff_sec
        
        for item in self._displacement.items():
            item[1][0] += self._robot_speed_x * timediff_sec
            item[1][1] += self._robot_speed_y * timediff_sec
            item[1][2] += self._robot_speed_theta * timediff_sec
    
    def _calc_radius(self, dX, dY):
        '''
        Calculate the radius of a delta in X and Y
        '''
        return sqrt(pow(dX, 2) + pow(dY,2))
        
    def _calc_angle(self, dX, dY):
        '''
        Calculate the angle between two deltas
        '''
        try:
            th = 0.0
            radius = self._calc_radius(dX, dY)
            
            if (dY < 0.0):
                if (dX == 0.0):
                    th = pi / 2.0;
                else:
                    th = arccos(-dX / radius)
            elif (dY > 0.0):
                if (dX == 0.0):
                    th = 3.0 / 2.0 * pi
                else:
                    th = 2.0 * pi - arccos(-dX / radius)
            elif (dY == 0.0):
                if (dX > 0.0):
                    th = 0.0
                else:
                    th = pi
        except:
            traceError('failed to calculate angle')
        
        return th
         
        
    def do_robot_have_balls(self):
        '''
        checks if the robot owns any ball based on the angle of ball handlers
        For simulation the visualizer will take care of claiming and releasing the ball
        Therefore always return False so that it will not be claimed
        '''
        return False
    
    def _trigger_event_on_ball_possession_change(self):
        '''
        call the ros service to update if there is a state change in ball possession. there can be two possible states: ('has ball' or 'has no ball') 
        '''        
        try:
            current_ball_possession_status = self.do_robot_have_balls()
        except:
            traceError('Cannot get current ball possession')

    def move_robot(self, x_velocity, y_velocity, theta_velocity):
        'moves the robot by distributing the speed to drive wheels in varying ratio based on the input'     
        try:               
            x_factor = [0.5, 0.5, -1.0]
            y_factor = [1.1547, -1.1547, 0.0] 
            theta_factor = [-1.0, -1.0, -1.034863]
            
            trace('calling move_robot')
            
            self._robot_speed_x = float(x_velocity)
            self._robot_speed_y = float(y_velocity)
            self._robot_speed_theta = float(theta_velocity)
            
            trace('move robot = %6.2f, %6.2f, %6.2f', x_velocity, y_velocity, theta_velocity)
            
            x_vel = self._robot_speed_x * cMotors.meter_per_sec_to_encoder_value
            y_vel = self._robot_speed_y * cMotors.meter_per_sec_to_encoder_value
            theta_vel = self._robot_speed_theta * cMotors.rad_per_sec_to_encoder_value
    
            left_motor_velocity = x_vel * x_factor[0] + y_vel * y_factor[0] + theta_vel * theta_factor[0]  
            right_motor_velocity = x_vel * x_factor[1] + y_vel * y_factor[1] + theta_vel * theta_factor[1]         
            rear_motor_velocity = x_vel * x_factor[2] + y_vel * y_factor[2] + theta_vel * theta_factor[2] 
            self._set_speed(left_motor_velocity, right_motor_velocity, rear_motor_velocity)
            
            'Update coupled Compass simulator'
            self._compass._update_angle_speed(theta_vel)
            
            'Update ball possession'
            self._trigger_event_on_ball_possession_change()
            
            'Update encoder readings'
            self._read_driving_encoders()
            
            trace('robot moved')
        except:
            traceError('Cannot move robot')
        
        
    def get_driving_speed(self):
        '''
        Fetch robot speed in meters per second
        '''        
        vx = 0.0
        vy = 0.0
        vtheta = 0.0
        
        try:            
            vx = self._robot_speed_x
            vy = self._robot_speed_y
            vtheta = self._robot_speed_theta 
            
            trace('get driving speed=%6.2f, %6.2f, %6.2f', vx, vy, vtheta)
        except:
            traceError('Failed to get driving speed')
        
        return vx, vy, vtheta
    
    def get_driving_displacement(self, uniqueID):
        '''
        Fetch the total displacement 
        '''
        dx = 0.0
        dy = 0.0
        dtheta = 0.0
        
        try:            
            if not self._displacement.has_key(uniqueID):
                self._displacement[uniqueID] = [0.0, 0.0, 0.0]
        
            dx = copy.copy(self._displacement[uniqueID][0])  
            dy = copy.copy(self._displacement[uniqueID][1])
            dtheta = copy.copy(self._displacement[uniqueID][2])
            
            self._displacement[uniqueID][0] = 0.0
            self._displacement[uniqueID][1] = 0.0
            self._displacement[uniqueID][2] = 0.0
            
            trace('get driving displacement = %6.2f, %6.2f, %6.2f', dx, dy, dtheta)
        except:
            traceError('Failed to get driving displacement')
            print traceback.format_exc()
            print sys.exc_info()[0]
        
        return dx, dy, dtheta
        
     
    def activate_ball_handling(self, speed_left=0, speed_right=0):
        'set the speed of ball handling cMotors'
        try:
            self._ball_speed_left = speed_left
            self._ball_speed_right = speed_right
            
            trace('activated ball handling')
        except:
            traceError('Failed to set active ball handling')
            
                                    
    def calibrate_ball_handling_angle(self):
        'calibrate the _angle of ball handling of both left and right cMotors'        
        pass
                  
class cKicker(object):
    '''
    Kicker class is used for controlling the shooter module 
    to kick or pass the ball based on the inputs such as shoot height and shoot speed
    '''
    
    BALL_SIM_FACTOR = 6.5 / 255.0
    
    def __init__(self):
        '''
        Initialize Kicker class
        Steps:
         - Open serial ports for I/O board
        '''        
        self._shoot_height = 0.0
        self._shoot_speed = 0.0 
        
        try:
            trace('Waiting for shoot to come online')
            rospy.wait_for_service('/simulator/shoot')
            self._sim_shoot_srv = rospy.ServiceProxy('/simulator/shoot', Shoot, True)
            trace('robot sim ball online')

            rospy.wait_for_service("s_set_own_robot_status")
            self._set_robot_status_srv = rospy.ServiceProxy("s_set_own_robot_status", set_own_robot_status, True)

            trace('Set own robot status service online.')

            self._srv_setKickPosition = rospy.Service('s_kick_position', s_peripheralsInterface_setKickPosition, self.setKickPosition)
            self._srv_setKickSpeed = rospy.Service('s_kick_speed', s_peripheralsInterface_setKickSpeed, self.setKickSpeed)

            # Set robot in play
            robot_status_req = set_own_robot_statusRequest()
            robot_status_req.robotStatus.status_type = rosMsgs.msg.s_robot_status.TYPE_INPLAY
            _ = self._set_robot_status_srv(robot_status_req)

        except Exception as e:
            traceError('failed to initialize cKicker')
            print traceback.format_exc()
            print sys.exc_info()[0]
            raise(e)

    def setKickPosition(self, request):
        response =  s_peripheralsInterface_setKickPositionResponse()

        'adjusts the shooter height by setting the given height to the stepper motor'
        self._shoot_height = float(request.kick_position)
        trace('shoot height: %8.4f' % (self._shoot_height))

        return response

    def setKickSpeed(self, request):
        'adjusts the shoot height and triggers the shooter with the set speed'

        response = s_peripheralsInterface_setKickSpeedResponse()

        try:
            self._shoot_speed = float(request.kick_speed)
            trace('shoot speed: %8.4f' % (self._shoot_speed))
            resp = self._sim_shoot_srv(self._shoot_speed * cKicker.BALL_SIM_FACTOR)
        except:
            traceError('failed to shoot with shoot speed: %8.4f' % (self._shoot_speed))

        return response
        
def main():
    'main module to start the driver services of all hardware peripherals'
    try:
        trace('Initing node Peripherals Sim')
        rospy.init_node('Peripherals_SIM', anonymous=False)

        kicker = cKicker()
        compass = cCompass()
        motors = cMotors(compass)        

        rospy.spin()
        
    except Exception, e:
       print str(e)
    finally:
       trace("Exiting peripherals driver server.")

if __name__ == '__main__':
    main()

