""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python

import rospy
import sys
import threading
import time

import roslib; roslib.load_manifest('peripheralsInterface') 
from peripheralsInterface.srv import s_homeKicker, s_peripheralsInterface_setKickPosition, s_peripheralsInterface_setKickSpeed

sys.path.append( "/home/robocup/falcons/code/packages/facilities/common/src" )
import FalconsEnv

def execute_service_call(service, service_name, arguments):
    communication_ok = False;
    while (True):
        try:
            service()
            if not communication_ok:
                print("Communication for " + service_name + " online again.")
            communication_ok = True
        except Exception as e:
            print("Could not execute " + service_name + ": " + str(e))
            communication_ok = False

robot_number = FalconsEnv.get_robot_num()
home_kicker = rospy.ServiceProxy("/teamA/robot" + str(robot_number) + "/s_home_kicker", s_homeKicker)   
set_kick_position = rospy.ServiceProxy("/teamA/robot" + str(robot_number) + "/s_kick_position", s_peripheralsInterface_setKickPosition)
set_kick_speed = rospy.ServiceProxy("/teamA/robot" + str(robot_number) + "/s_kick_speed", s_peripheralsInterface_setKickSpeed)

t = threading.Thread(target=execute_service_call, args=(home_kicker, "Home kicker", None))
t.daemon=True
t.start()
#execute_service_call(set_kick_position, (0))
#execute_service_call(set_kick_speed, (0))

try:
    while (True):
        time.sleep(1.0)
except KeyboardInterrupt:
    pass