""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Analysis node on coach laptop.
# Listens to all diagnostics topics, adds a few new analysis topics next to it.
#
# Jan Feitsma, 2016-03-05



import argparse
import roslib
import rospy
roslib.load_manifest('analyzer') 

# analyzer imports
from settings import *
from team import AnalyzeTeam

        
class AnalyzerRealtime():
    def __init__(self, publish=True):
        # initialize node
        rospy.init_node('analyzer', anonymous=False)
        # construct all objects
        self.anaTeam = AnalyzeTeam(publish)
        
    def run(self):
        rate = rospy.Rate(WRITE_FREQUENCY)
        while not rospy.is_shutdown():
            self.anaTeam.update()
            rate.sleep()    
            
            

if __name__ == '__main__':
    # argument parsing
    parser     = argparse.ArgumentParser(description='coach realtime analyzer')
    parser.add_argument('--nopublish', help='do not construct ROS publishers', action='store_true')
    args, leftovers = parser.parse_known_args()

    # run
    a = AnalyzerRealtime(publish=(not args.nopublish))
    a.run()
    
        
