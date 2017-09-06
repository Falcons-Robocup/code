""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python


# Description: commandChannel Proxy
#
# commandChannelProxy runs on robot and works together with the commandChannel Listener running on Coach
# The commandChannelListener listens on Coach topic /g_commandChannelToRobots
# The topic messages are transformed in UDP multicast transmission
# This script acts as commandChannelProxy, catches the messages and republish on local robot  /g_commandChannelIn topic
# Other processes on robot can subscribe to topic and filter those messages they are interested in
# 

import os
import rospy
import roslib
roslib.load_manifest('robotControl')

from rosMsgs.msg import t_action
from rosMsgs.msg import t_commandChannel
import pickle

import socket
import asyncore  #to handle asynchronous socket communication
import struct
import sys
import hashlib

import FalconsTrace

class UDPcommunicator( asyncore.dispatcher ):
   def __init__(self, multicastIP, multicastPort, receiveCallBack ):
      asyncore.dispatcher.__init__(self)
      self.multicastIP=multicastIP
      self.multicastPort=multicastPort
      self.receiveCallBack=receiveCallBack

      #create_socket(family, type)
      self.create_socket( socket.AF_INET, socket.SOCK_DGRAM)
      # Set the time-to-live for messages to 1 so they do not go past the
      # local network segment.
      ttl = struct.pack('b', 1)
      self.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl)

      aton = socket.inet_aton(self.multicastIP)
      mreq = struct.pack('4sL', aton, socket.INADDR_ANY)

      self.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
      self.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  #enable reuse of address/port by others (especially simulator environment where multiple virtual robots are listening to same ip/port address

      # Bind to the server address/port 
      self.bind( ('', self.multicastPort))

      # Set a timeout so the socket does not block indefinitely when trying
      # to receive data.
      self.settimeout(0.2)

   def send(self, data):
      print "Sending to UDP"
      return self.sendto( data, (self.multicastIP, self.multicastPort) )

   def handle_read(self):
      print "Receiving from UDP"
      self.receiveCallBack( self.recvfrom(1024) )


def cb_commandChannelSend(data):
        #SEND TO UDP
      
        data.checksum=0  #initialize before creating hash
        print("New topic message received: %s" % data )
        myHash=hashlib.new('md5')
        myHash.update( pickle.dumps(data) )
        hashString=myHash.hexdigest()[:4]  #only use first 4 chars

        data.checksum=hashString

        sent=udpComm.send( pickle.dumps(data) )
        print("UDP sent result: %s" %sent )

def cb_commandChannelReceive( (data,address)):
        #print "Received data %s  from %s" % ( data, address )

        UDPmsg=pickle.loads( data )
        checksum=UDPmsg.checksum
        UDPmsg.checksum=0  #back to initial value to calculate the correct checksum again

        myHash=hashlib.new('md5')
        myHash.update( pickle.dumps( UDPmsg ) )
        hashString=myHash.hexdigest()[:4]  #only use first 4 chars
        if hashString == checksum:
           UDPmsg.checksum="OK"
           # only proxy when checksum is OK
           commandChannelFromCoachHandle.publish( UDPmsg ) #send to ROS topic


if __name__ == '__main__':  

        ROS_NAMESPACE = os.environ['ROS_NAMESPACE']
 	#FalconsTrace.trace( 'ROS NAMESPACE=%s'  % ROS_NAMESPACE )     

        udpComm=UDPcommunicator( '224.3.29.71', 10000, cb_commandChannelReceive)  
  	rospy.init_node('commandChannelUDPProxy')
        commandChannelFromCoachHandle = rospy.Publisher('g_commandChannelFromCoach', t_commandChannel, queue_size=3)

        #everything published on g_commandChannelToRobots will be picket up and sent to UDP multicast
        print "Listening to g_commandChannelToCoach"
        rospy.Subscriber('g_commandChannelToCoach' , t_commandChannel , cb_commandChannelSend)

        loopRate=rospy.Rate(30)  # 30 Hz

        #print "Entering LOOP"
	while not rospy.is_shutdown():
            asyncore.loop( count=1 )
            loopRate.sleep()





 





        
