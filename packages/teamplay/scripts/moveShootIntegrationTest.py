""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# moveShootIntegrationTest
# purpose: test shoot on goal after some turningmovements
#
# 
# History:
# v 1.00 MKOE 20160802 - creation, for Stans tests, forked from moveShootAlignmentTest
#                        requirements spec by Stan
#
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
   requestReset() 
   sleep(0.1)
   requestAction( "stop", "once")
   sleep(0.1)
   sys.exit()


if __name__ == '__main__':  
    
   try:
        signal.signal(signal.SIGINT, signal_term_handler)
        rospy.init_node('moveShootIntegrationTest')

        print "FYI: All moves in this script will happen with default Phi for position tolerance:"
        os.system("cd $HOME/falcons/code/packages/teamplay/include/int/actions; grep PHI cAbstractAction.hpp")

        print "FYI: All possible shoot types are as follows:"
        os.system("cd $HOME/falcons/code/packages/teamplay/include/int/actions; grep defaultShootTypes cAbstractAction.hpp")
        print "This program will loop until you press control-C !"
        print

        robotNumber = raw_input("Enter robot number: ")

        POIChoices=[ "P_CENTER" , "P_OPP_PENALTYAREA_CORNER_LEFT", "coord:4.0,3.0" ]
        count=1
        print "Enter robot start location:"
        for POIname in POIChoices:
           print "%s. %s" % ( count , POIname )
           count=count+1

        myPOIChoice= raw_input("answer: ")
        POIindex = int ( myPOIChoice ) - 1
        POIname= POIChoices[  POIindex ]

    
	teamName="teamA"
        rosNamespace="/" + teamName + "/robot" + robotNumber + "/"


        fullServiceName=rosNamespace + "/s_tp_input_command" 

        tpControl_srv=rospy.ServiceProxy( fullServiceName, s_tp_input_command, persistent=True)
        reqMsg=s_tp_input_commandRequest()

        while 1==1:
    
           answer = raw_input("Put ball in robot, press enter to move to start position.")

           requestAction( "move target=%s facing=P_OPP_GOALLINE_CENTER" % ( POIname ) , "whileRunning" )

           answer = raw_input("Robot reached starting position, press enter to continue with next step")

           requestAction( "move target=P_OPP_PENALTY_SPOT facing=P_OPP_GOALLINE_CENTER"
 , "whileRunning" )

           answer = raw_input("Press enter to shoot")

           requestAction( "shoot shootType=passTowardsNearestTeammember", "once")
           requestAction( "stop", "once")


 


   except KeyboardInterrupt:
      #quit
      requestReset() 
      shutdownHook()
      sys.exit()

 



