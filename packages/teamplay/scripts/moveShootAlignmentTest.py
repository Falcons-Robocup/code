""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# moveShootAlignmentTest
# purpose: test shoot on goal after some turningmovements
#
# 
# History:
# v 1.00 MKOE 20160629 - creation

#
# 
#

import sys, os
import socket   # to get hostname
import rospy
import numpy
import time
import random
import roslib; roslib.load_manifest('teamplay')
import subprocess
import FalconsEnv
import FalconsTrace
import FalconsMath
import EnvironmentField      #defined in pymodule in environment facility
from EnvironmentField import *

# http://wiki.ros.org/rospy/Overview/Services
from teamplay.srv import s_tp_input_command, s_tp_input_commandRequest

TURTLEROOT=os.environ['TURTLEROOT']

import argparse
import time, datetime


import signal

def signal_term_handler(signal, frame):
  '''Handles KeyboardInterrupts to ensure smooth exit'''
  rospy.logerr('User Keyboard interrupt')
  shutdownHook()
  sys.exit(0)


def requestReset():
   print "Reset"
   reqMsg.command='reset'
   response=tpControl_srv( reqMsg )
   print response.result


def requestAction( actionString, repeat ):
   print "Action: " + actionString
   reqMsg.command='action ' + actionString
   response=tpControl_srv( reqMsg )
   if repeat == "once":
      return

   if repeat == "whileRunning":
      while response.result == "RUNNING":
         response=tpControl_srv( reqMsg )
         #print response.result
         time.sleep(1.0/10.0)
      return

   return

def shutdownHook():
   requestAction( "stop", "once")
   sleep(0.1)
   sys.exit()


if __name__ == '__main__':  
    
   try:
        signal.signal(signal.SIGINT, signal_term_handler)
        rospy.init_node('moveShootAlignmentTest')

        robotNumber = raw_input("Enter robot number: ")
        angleThreshold = raw_input("Enter angleThreshold: ")

        POIChoices=[ "P_CENTER" , "P_OPP_PENALTYAREA_CORNER_LEFT", "P_OPP_PENALTYAREA_CORNER_RIGHT" ]
        count=1
        for POIname in POIChoices:
           print "%s. %s" % ( count , POIname )
           count=count+1

        myPOIChoice= raw_input("Enter robot target location POI: ")
        POIindex = int ( myPOIChoice ) - 1
        POIname= POIChoices[  POIindex ]
    
	teamName="teamA"
        rosNamespace="/" + teamName + "/robot" + robotNumber + "/"


        fullServiceName=rosNamespace + "/s_tp_input_command" 

        tpControl_srv=rospy.ServiceProxy( fullServiceName, s_tp_input_command, persistent=True)
        reqMsg=s_tp_input_commandRequest()
    
        answer = raw_input("Put ball in robot, press enter to continue.")

        requestAction( "move target=%s facing=P_OPP_PENALTY_SPOT angleThreshold=%s" % ( POIname , angleThreshold) , "whileRunning" )

        requestAction( "move target=robot facing=P_OPP_CORNER_LEFT angleThreshold=%s" % angleThreshold , "whileRunning" )
        requestAction( "move target=robot facing=P_OPP_CORNER_RIGHT angleThreshold=%s" % angleThreshold , "whileRunning" )
        requestAction( "move target=robot facing=P_OPP_GOALLINE_CENTER angleThreshold=%s" % angleThreshold , "whileRunning" )
        requestAction( "shoot shootType=shootTowardsGoal", "once")
        requestAction( "stop", "once")

        answer = raw_input("Put ball in robot, press enter to continue.")

        requestAction( "move target=robot facing=P_OPP_CORNER_RIGHT angleThreshold=%s" % angleThreshold , "whileRunning" )
        requestAction( "move target=robot facing=P_OPP_CORNER_LEFT angleThreshold=%s" % angleThreshold , "whileRunning" )
        requestAction( "move target=robot facing=P_OPP_GOALLINE_CENTER angleThreshold=%s" % angleThreshold , "whileRunning" )
        requestAction( "shoot shootType=shootTowardsGoal", "once" )
        requestAction( "stop", "once")
        #requestReset()    #disabled for now because this generates *** Error in `/home/robocup/falcons/code/packages/teamplay/bin/teamplay_main': free(): invalid pointer: 0x000000000189e8cf ***


   except KeyboardInterrupt:
      #quit
      shutdownHook()
      sys.exit()





