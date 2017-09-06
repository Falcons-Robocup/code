""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
import roslib; roslib.load_manifest('compass') 
import rospy
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

import FalconsEnv
from compass.srv import *
from compass.cfg import compassConfig
import socket
import Pyro4
import os
from os.path import expanduser
from subprocess import call
from numpy.ma.core import abs, mod
from logging import exception
 

class readCompass(object):
    '''
    Class for reading the compass of the robot
    '''
    
    def __init__(self, uri, port):
        '''
        Initialize the readCompass class
        '''
        
        # in ROS, nodes are unique named. If two nodes with the same
        # node are launched, the previous one is kicked off. The 
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaenously.
        rospy.init_node('Compass', anonymous=False)
        
        # Ensure compass angle has default value
        self._angle = 0;
        self._homegoalAngle = 0;
        self._CYAN_is_leftSide = True
        
        # Load YAML file from falconsTesting
        self._loadYAMLConfig()
        
        # Initialize the connection with the Remote Object
        self._compass = Pyro4.Proxy("PYRO:compass@" + str(uri) + ":" + str(port))
        
        # Set functionality to enable dynamic reconfigure
        self._reconfigserver = DynamicReconfigureServer(compassConfig, self._reconfigure)
        
    def _reconfigure(self, config, level):
        '''
        Reconfigure the Compass Node
        '''
        self._homegoalAngle = config['homegoal_angle']
        self._CYAN_is_leftSide = config['CYAN_IS_LEFT_SIDE']
        return config
    
    def _readCompass(self):
        '''
        Read compass from Pyro4 API
        '''
        try:
            self._angle = self._compass.get_angle()
        except:
            print ("Error: Failed to get compass value")
        
    def _loadYAMLConfig(self):
        '''
        Read YAML config in directory falconsTesting and execute rosparams
        '''
        home = expanduser("~")
        command = "rosparam load " + home + "/falcons/code/config/Compass.yaml"
        call(command, shell=True)
                    
    
    def cb_ReadCompass(self, req):
        '''
        making api call to the middleware which resides outside ROS
        output parameters: 
         - theta
        '''     
        self._readCompass()
        
        resp = CompassResponse()
        
        if self._CYAN_is_leftSide:
            resp.theta.data = abs(mod(360.0 - self._angle + self._homegoalAngle, 360.0));
        else:
            resp.theta.data = abs(mod(360.0 - self._angle + self._homegoalAngle + 180.0, 360.0));
        return resp
        
        
    def cb_ReadRawCompass(self, req):
        '''
        making api call to the middleware which resides outside ROS
        output parameters: 
         - theta
        '''     
        self._readCompass()
        
        resp = Compass_rawResponse()
        resp.theta.data = self._angle
        return resp
        
    def start_listening(self):
        '''
        Enable listening to the Compass services
        '''       
        # Enable compass services
        service_RawCompass = rospy.Service('s_get_compass_raw', Compass_raw, self.cb_ReadRawCompass)
        service_Compass = rospy.Service('s_get_compass', Compass, self.cb_ReadCompass)
        
    
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        
if __name__ == '__main__':

    ip_address = FalconsEnv.get_ip_address()
    portnum = FalconsEnv.get_pyro_port()
    FalconsEnv.info() # put a oneliner for debugging on stdout

    compass = readCompass(ip_address, portnum)
    compass.start_listening()
