""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python

# History:
# MKOE 20150707 - implement teamplay-reasoning diagnostics info to be sent do diag logger
# JFEI?  - expanded with other stuff
# MKOE 20160328 - add reasoning task to message and logger

import roslib; roslib.load_manifest('diagnostics') 
import rospy

import FalconsEnv
from FalconsTrace import trace
from diagnostics.srv import *
from rosMsgs.msg import *
from rosSrvs.srv import *
from worldModel.srv import *
from pathPlanning.srv import s_pathplanning_get_active
from shootPlanning.srv import s_shootplanning_get_active

import os
import subprocess
import collections
import time
from random import random
 

class Diagnostics(object):
    '''
    Class for gathering robot diagnostics and publish to g_diagnostics.
    '''
    
    def __init__(self, frequency = 1.0):
        '''
        Initialize the class
        '''
        
        # in ROS, nodes are unique named. If two nodes with the same
        # node are launched, the previous one is kicked off. The 
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'node so that multiple listeners can
        # run simultaenously.
        trace("constructing Diagnostics object")
        rospy.init_node('diagnostics', anonymous=True)

        # Frequency
        self._frequency = frequency
        
        # The data.
        self._data = t_diagnostics()
        
        # Fill in robot number
        self._robotnum = FalconsEnv.get_robot_num()
        self._data.robotnum = self._robotnum
        
        # Provide the services for external clients to set their internal data
        trace("setting up service providers")
        service_pathpl = rospy.Service('s_set_pathplanning_diagnostics', s_set_pathplanning_diagnostics, self.cb_pathplanning)
        service_vision = rospy.Service('s_set_vision_diagnostics', s_set_vision_diagnostics, self.cb_vision)
        service_wmsync = rospy.Service('s_set_wmsync_diagnostics', s_set_wmsync_diagnostics, self.cb_wmsync)
        service_reasoning = rospy.Service('s_set_tp_reasoning_diagnostics', s_set_tp_reasoning_diagnostics, self.cb_tp_reasoning)
        
        # Subscribe to topics
        trace("subscribing to topics")
        self._sub_target = rospy.Subscriber("g_target", t_target, self.cb_target)
        self._sub_speed  = rospy.Subscriber("g_robotspeed", t_robotspeed, self.cb_speed)
        self._sub_role   = rospy.Subscriber("g_role", t_role, self.cb_role)
        self._sub_state  = rospy.Subscriber("g_state", t_state, self.cb_state)
        self._sub_action = rospy.Subscriber("g_action", t_action, self.cb_action)
        self._sub_task   = rospy.Subscriber("g_task", t_task, self.cb_task)
        self._sub_refbox = rospy.Subscriber("g_refbox_local",t_refbox_local,self.cb_refbox)
        
        # Subscribe to services
        trace("subscribing to services")
        self._service_cache = {}
        servicename = 's_get_active_robots'
        self._wm_active_service = self.get_service(servicename, get_active_robots)
        #servicename = 's_actionhandler_get_active'
        #self._ah_active_service = self.get_service(servicename, s_actionhandler_get_active)
        servicename = 's_pathplanning_get_active'
        self._pp_active_service = self.get_service(servicename, s_pathplanning_get_active)
        servicename = 's_shootplanning_get_active'
        self._sp_active_service = self.get_service(servicename, s_shootplanning_get_active)
        servicename = 's_get_teammembers'
        self._wm_teammembers_service = self.get_service(servicename, get_teammembers)
        servicename = 's_get_opponents'
        self._wm_opponents_service = self.get_service(servicename, get_opponents)
        servicename = 's_get_ball_location'
        self._wm_ball_location_service = self.get_service(servicename, get_ball_location)
        servicename = 's_get_ball_possession'
        self._wm_ball_possession_service = self.get_service(servicename, get_ball_possession)
        servicename = 's_get_own_location'
        self._wm_own_location_service = self.get_service(servicename, get_own_location)

        # Setup resulting topic publisher
        trace("setting up output topic")
        self._topic_handle = rospy.Publisher('g_diagnostics', t_diagnostics, queue_size = 3)
             
    def get_service(self, *args):
        """
        Return a service client connection, create if not yet existing.
        """
        assert(isinstance(args, collections.Hashable))
        key = args
        if self._service_cache.has_key(key):
            # cache hit
            trace('cache hit')
            service = self._service_cache[key]
        else:
            # cache miss - wait, construct client and store in cache
            trace('waiting for service', args[0])
            rospy.wait_for_service(args[0])
            service = rospy.ServiceProxy(*args, persistent=True)
            self._service_cache[key] = service
        return service
        
    def update_lf(self):
        """Low-frequent diagnostics sampling."""
        # TODO: also split HF from LF data channels to save on bandwidth
        # generic health checks
        cmd_str = "healthcheck_eonly"
        output = subprocess.check_output(cmd_str, stderr=subprocess.STDOUT, shell=True)
        self._data.healthcheck = output
        # get battery status
        cmd_str = "get_battery_voltage"
        output = subprocess.check_output(cmd_str, stderr=subprocess.STDOUT, shell=True)
        self._data.hardware.battery_voltage = float(output)
        # get CPU load
        cmd_str = "cat /proc/loadavg"
        output = subprocess.check_output(cmd_str, stderr=subprocess.STDOUT, shell=True)
        self._data.hardware.cpu_load = float(output.split(' ')[0])
        # network statistics part 1 - parse output of ifconfig
        cmd_str = "ifconfig -a"
        output = subprocess.check_output(cmd_str, stderr=subprocess.STDOUT, shell=True)
        # don't know which wlan id (wlan0 or wlan1) so let's search
        in_wlan = False
        for line in output.splitlines():
            # new adapter?
            if len(line) and (line[0] != ' '): 
                in_wlan = "wlan" in line
            if in_wlan:
                pos = line.find("RX packets")
                if pos >= 0:
                    self._data.network.recv_packets = int(line[(pos+11):].split(' ')[0])
                pos = line.find("TX packets")
                if pos >= 0:
                    self._data.network.sent_packets = int(line[(pos+11):].split(' ')[0])
                pos = line.find("RX bytes")
                if pos >= 0:
                    self._data.network.recv_bytes = int(line[(pos+9):].split(' ')[0])
                pos = line.find("TX bytes")
                if pos >= 0:
                    self._data.network.sent_bytes = int(line[(pos+9):].split(' ')[0])
        # network statistics part 2 - parse output of iwconfig
        cmd_str = "iwconfig"
        output = subprocess.check_output(cmd_str, stderr=subprocess.STDOUT, shell=True)
        # don't know which wlan id (wlan0 or wlan1) so let's search
        in_wlan = False
        for line in output.splitlines():
            # new adapter?
            if len(line) and (line[0] != ' '): 
                in_wlan = "wlan" in line
            if in_wlan:
                pos = line.find("Link Quality")
                if pos >= 0:
                    self._data.network.quality = line[(pos+13):].split(' ')[0]

    def update_hf(self):
        """High-frequent diagnostics sampling."""
        trace("update_hf start")
        # get inplay switch status
        response = self._wm_active_service() 
        # TODO: why does this WM service give characters, not a list of ints?
        trace("update_hf inplay")
        self._data.inplay = self._robotnum in [ord(r) for r in response.active_robots]
        if 0: # JFEI 20160105 somehow sometimes a bunch of processes die... may have to do with deadlock... disabling these service calls
            # get pathplanning activity status
            trace("update_hf pp_active")
            response = self._pp_active_service()
            self._data.pathplanning.active = response.active
            # get actionhandler activity status
            # JFEI the next 2 lines are disabled due to ticket #258
            # and possible trouble with circular dependencies since Michel has implemented logging of 'task' (?)
            #response = self._ah_active_service()
            #self._data.teamplay.actionhandler_active = response.active
            # get shootplanning activity status
            trace("update_hf sp_active")
            response = self._sp_active_service()
            self._data.shootplanning.active = response.active

        # workaround for Visualizer reporting "LOST CONNECTION" 
        # to each simulated robot:
        # * the error is derived from some number which is supposed to be 
        #   changing always
        # * I chose vision FPS
        #   * it would be nicer to do a hash check on a log sample 
        #     but that may turn out to be too costly
        #   * I can trust the vision module to e.g. not stop transmitting when  
        #     robot is set to "out of play"
        # * but in simulation FPS is not set so it does not change from 0.0, 
        #   since we do not have a simulator counterpart of the vision module
        # * we SHOULD have such a simulated vision module (stub), which sends 
        #   similar data to WM and diagnostics like vision would do
        # * until we have that, let's put some noise on FPS here
        if FalconsEnv.get_simulated():
            self._data.vision.fps = min(abs(random() * 0.001), 0.1)

        # get WorldModel details -- teammembers
        trace("update_hf WorldModel start")
        self._data.worldmodel.friends = []
        response = self._wm_teammembers_service()
        for robot in response.teammembers.RobotArray:
            p = posvel()
            p.x = robot.Pos.x
            p.y = robot.Pos.y
            p.orient = robot.Pos.theta
            p.vx = robot.Vel.x
            p.vy = robot.Vel.x
            # ignore robot ID and confidence
            self._data.worldmodel.friends.append(p)
        # get WorldModel details -- opponents
        self._data.worldmodel.enemies = []
        response = self._wm_opponents_service()
        for robot in response.opponents.RobotArray:
            p = posvel()
            p.x = robot.Pos.x
            p.y = robot.Pos.y
            p.orient = robot.Pos.theta
            p.vx = robot.Vel.x
            p.vy = robot.Vel.x
            # ignore robot ID and confidence
            self._data.worldmodel.enemies.append(p)
        # get WorldModel details -- own position (vx and vy unused)
        response = self._wm_own_location_service() 
        self._data.worldmodel.ownpos.x = response.ownRobot.Pos.x
        self._data.worldmodel.ownpos.y = response.ownRobot.Pos.y
        self._data.worldmodel.ownpos.orient = response.ownRobot.Pos.theta
        # get WorldModel details -- ball presence and location
        response = self._wm_ball_location_service() 
        nball = len(response.ballPos.BallArray)
        self._data.worldmodel.ballPresent = (nball > 0)
        if nball:
            # only set first entry
            self._data.worldmodel.ballpos.x = response.ballPos.BallArray[0].Pos.x
            self._data.worldmodel.ballpos.y = response.ballPos.BallArray[0].Pos.y
        else:
            self._data.worldmodel.ballpos.x = float('nan')
            self._data.worldmodel.ballpos.y = float('nan')
        # get WorldModel details -- ball possession
        response = self._wm_ball_possession_service() 
        self._data.worldmodel.ballPossession = response.possession
        self._data.worldmodel.has_ball = False
        if (response.possession.type == BallPossession.TYPE_TEAMMEMBER):
	        if (response.possession.robotID == self._robotnum):
	            self._data.worldmodel.has_ball = True
        trace("update_hf end")
        
    def cb_target(self, msg):
        self._data.pathplanning.target.x = msg.x
        self._data.pathplanning.target.y = msg.y
        self._data.pathplanning.target.orient = msg.phi
        self._data.pathplanning.target.vx = msg.vx
        self._data.pathplanning.target.vy = msg.vy
        
    def cb_speed(self, msg):
        self._data.pathplanning.vx = msg.vx
        self._data.pathplanning.vy = msg.vy
        self._data.pathplanning.vphi = msg.vphi
        
    def cb_role(self, msg):
        self._data.teamplay.role = msg.role

    def cb_task(self, msg):
        self._data.teamplay.task = msg.task
        
    def cb_state(self, msg):
        self._data.teamplay.state = msg.state
        
    def cb_action(self, msg):
        self._data.teamplay.action = str(msg.action)
        
    def cb_pathplanning(self, req):
        """
        Service provided for pathplanning to fill actively.
        Note also the topic subscriptions (g_target, g_robotspeed).
        """
        self._data.pathplanning.subtarget = req.subtarget;
        resp = s_set_pathplanning_diagnosticsResponse()
        return resp

    def cb_refbox(self, msg):
        self._data.command  = msg.command
        self._data.dtime    = rospy.Time.now()

    def cb_vision(self, req):
        self._data.vision = req.data;
        # dummy return
        resp = s_set_vision_diagnosticsResponse()
        return resp

    def cb_wmsync(self, req):
        self._data.network.packet_loss = req.packet_loss;
        # dummy return
        resp = s_set_wmsync_diagnosticsResponse()
        return resp

    def cb_tp_reasoning(self, req):
        self._data.teamplay.reasoning = req.data;
        # dummy return
        resp = s_set_tp_reasoning_diagnosticsResponse()
        return resp
    
    def run(self):
        '''
        Spin, publish the data gathered via services.
        '''       
        rate = rospy.Rate(self._frequency)
        counter = 0
        while not rospy.is_shutdown():
            trace("iterate")
            # passive update via external service calls and topic subscriptions
            # so standard ROS stuff
            
            # active update by polling e.g. WorldModel
            # high-frequent: 10Hz
            self.update_hf()
            # low-frequent
            if counter % 30 == 0:
                trace("LF update start")
                try: # might fail in sim mode
                    self.update_lf()
                    trace("LF update OK")
                except:
                    trace("LF update FAIL")
                    pass
            self._topic_handle.publish(self._data)
            rate.sleep()
            counter += 1


if __name__ == '__main__':
    diagnostics = Diagnostics(frequency = 10.0)
    diagnostics.run()
    
    
