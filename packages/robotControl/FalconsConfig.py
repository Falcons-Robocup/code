""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import roslib
import dynamic_reconfigure.client
import time
import socket

def get_on_real_robot():
    """Return true if called on real robot (by looking at hostname)."""
    return "FALCON" in socket.gethostname()


class FalconsConfig:
    def __init__(self, rosPrefix):

        # ROS namespace
        self.prefix = rosPrefix
        
        # Init configurators
        self.configClient = {}

        # There are some extra nodes in real mode
        if get_on_real_robot():
            self.configClient["worldModelNode/localization"] = dynamic_reconfigure.client.Client(self.prefix + "worldModelNode/localization", timeout=2)
            self.configClient["worldModelNode/obstacleTracker"] = dynamic_reconfigure.client.Client(self.prefix + "worldModelNode/obstacleTracker", timeout=2)
            #self.configClient["peripheralsInterface/motors"] = dynamic_reconfigure.client.Client(self.prefix + "peripheralsInterface/motors", timeout=2)
        #self.configClient["PathPlanningNode"] = dynamic_reconfigure.client.Client(self.prefix + "PathPlanningNode", timeout=2)

        # PathPlanningNode seems not responsive ... TODO
        
        # Store initial configurations, so some scenarios can easily reset
        self.configDefault = {}
        for module in self.configClient.keys():
            self.configDefault[module] = self.configClient[module].get_configuration(timeout=1)
                
    def show(self, module):
        print self.configClient[module].get_configuration()

    def get(self, module):
        return self.configClient[module].get_configuration()

    def set(self, module, argDict):
        if self.configClient.has_key(module):
            self.configClient[module].update_configuration(argDict)

    def restore(self, module):
        if self.configClient.has_key(module):
            self.configClient[module].update_configuration(self.configDefault[module])

    def restoreAll(self):
        for module in self.configClient.keys():
            self.restore(module)

