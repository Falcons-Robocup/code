""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
import roslib
roslib.load_manifest('rosMsgs')
roslib.load_manifest('worldModel')
import rospy

import sys
sys.path.insert(0, '/usr/lib/pymodules/python2.7/')

from multiprocessing import Process, Pipe

from rosMsgs.msg import t_target
from rosMsgs.msg import t_robotspeed
from rosMsgs.msg import t_motor_pid_params

from worldModel.srv import * 

import socket
import os, sys
import traceback
import datetime
from os.path import expanduser
from subprocess import call

import numpy as np

import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import matplotlib.animation as anim
from PyQt4 import QtCore,QtGui

import time
import math

from multiprocessing.sharedctypes import Value, Array
from multiprocessing import Process, Lock
from ctypes import Structure, c_double

graph_width = 100
draw_freq = 1.0 / 20.0
frequency = 30.0
_dt = 1.0 / frequency

ROBOT_NR = "1"

# Make "defines" for the name of each line that can be plotted

# g_robotspeed is published by pathplanning towards the motors
GRSVX    = 'g_robotspeed.vx' 
GRSVY    = 'g_robotspeed.vy' 
GRSVPHI  = 'g_robotspeed.vphi' 

# t_motor_pid_params is the output of each motor board
GMPPID   = 't_motor_pid_params.motorId'
GMPPSP   = 't_motor_pid_params.setPoint'
GMPPMV   = 't_motor_pid_params.measuredValue'
GMPPERR   = 't_motor_pid_params.error'
GMPPI   = 't_motor_pid_params.integral'
GMPPD   = 't_motor_pid_params.derivative'
GMPPPIDOUT   = 't_motor_pid_params.pidOutput'
GMPPDISP   = 't_motor_pid_params.displacementEncTicks'
GMPPVEL   = 't_motor_pid_params.velocity'
GMPPVELERR   = 't_motor_pid_params.velocityError'

# g_motor_robot_speed is the robot's velocity according to peripheralsInterface
GMRSVX    = 'g_motor_robot_speed.vx' 
GMRSVY    = 'g_motor_robot_speed.vy' 
GMRSVPHI  = 'g_motor_robot_speed.vphi' 

# g_target is the setpoint position for pathPlanning
GTPX     = 'g_target.pos.x' 
GTPY     = 'g_target.pos.y' 
GTPTHETA = 'g_target.pos.theta' 
GTVX     = 'g_target.vel.x' 
GTVY     = 'g_target.vel.y' 
GTVZ     = 'g_target.vel.z' 

# s_get_own_location is the robot's own location according to WorldModel
SOLX     = 's_get_own_location.Pos.x'
SOLY     = 's_get_own_location.Pos.y'
SOLTHETA = 's_get_own_location.Pos.theta'
SOVX     = 's_get_own_location.Vel.x'
SOVY     = 's_get_own_location.Vel.y'
SOVTHETA = 's_get_own_location.Vel.theta'
SOAX     = 'curr.acceleration.x'
SOAY     = 'curr.acceleration.y'
SOATHETA = 'curr.acceleration.theta'

########################################################
# (UN)COMMENT VALUES HERE TO (DE)ACTIVATE THE PLOTTING #
########################################################
_values_name = [
       # t_robotspeed / g_robotspeed -> Output of PP, Input of PI
       GRSVX,
       GRSVY,
       #GRSVPHI,

       # t_motor_pid_params -> Output of motor boards
       #GMPPID,
       #GMPPSP,
       #GMPPMV,
       #GMPPERR,
       #GMPPI,
       #GMPPD,
       #GMPPPIDOUT,
       #GMPPDISP,
       #GMPPVEL,
       #GMPPVELERR,

       # t_robotspeed / g_motor_robot_speed -> Output of PI
       #GMRSVX,
       #GMRSVY,
       #GMRSVPHI,

       # g_target
       GTPX,
       GTPY,
       #GTPTHETA,
       #GTVX,
       #GTVY,
       #GTVZ,

       # s_get_own_location
       SOLX,
       SOLY
       #SOLTHETA,
       #SOVX,
       #SOVY,
       #SOVTHETA
       #SOAX
       #SOAY
       #SOATHETA
      ]
########################################################
########################################################




class LineGraph(object):
   def __init__(self, values_arr, graph_names):

      # Create three Axes. One for X, Y and Phi.
      # See http://matplotlib.org/faq/usage_faq.html
      self._f, (self._ax1, self._ax2, self._ax3) = plt.subplots(3, sharex=True, sharey=False)
      self._f.canvas.set_window_title('Harry Plotter')

      # Fine-tune figure; make subplots close to each other and hide x ticks for
      # all but bottom plot.
      self._f.subplots_adjust(hspace=0.05)
      #plt.setp([a.get_xticklabels() for a in self._f.axes[:-1]], visible=False)

      #Optional: Maximize window on startup
      #mng = plt.get_current_fig_manager()
      #mng.resize(*mng.window.maxsize())

      # Save the shared values array
      self._values = values_arr
      self._values_name = graph_names

      # This dictionary contains all Line2D objects
      # Key = String value (e.g. SOLX)
      self._plots = {}

      # This dictionary represents the data that is plotted for each Line2D
      # Key = String value (e.g. SOLX)
      self._plotdata = {}

      # Build all plots for each of the Axes
      for plot in self._values_name:
         if plot.endswith('x'):
            # Add to Ax1
            self._plotdata[plot] = [ self._values[self._values_name.index(plot)] ]
            self._plots[plot], = self._ax1.plot( self._plotdata[plot], label=plot )
         elif plot.endswith('y'):
            # Add to Ax2
            self._plotdata[plot] = [ self._values[self._values_name.index(plot)] ]
            self._plots[plot], = self._ax2.plot( self._plotdata[plot], label=plot )
         else:
            # Add to Ax3
            self._plotdata[plot] = [ self._values[self._values_name.index(plot)] ]
            self._plots[plot], = self._ax3.plot( self._plotdata[plot], label=plot )

      self._ax1.legend()
      self._ax2.legend()
      self._ax3.legend()

      self._ax1.set_xlim(0, graph_width)

      # Optional: Hook a callback function on the window resize event
      self._f.canvas.mpl_connect('resize_event', self.onresize)

      line_ani = anim.FuncAnimation(self._f, self.update_lines, interval=10, blit=False)
      plt.show()

   def update_lines(self, num):

      returnVal = []

      # Find minimum and maximum for all three Axes
      min1 = 0.0
      max1 = 0.0
      min2 = 0.0
      max2 = 0.0
      min3 = 0.0
      max3 = 0.0

      for plot in self._values_name:

         # Pop and append
         if len( self._plotdata[plot] ) == graph_width:
            self._plotdata[plot].pop(0)
         self._plotdata[plot].append( self._values[self._values_name.index(plot)]  )

         # Set data
         self._plots[plot].set_data( [i for i in range( len(self._plotdata[plot]) )], self._plotdata[plot] )
         returnVal.append(self._plots[plot])

         # Get current maximum / minimum
         curmin = min( self._plotdata[plot] )
         curmax = max( self._plotdata[plot] )

         if plot.endswith('x'):
            if curmin < min1:
               min1 = curmin
            if curmax > max1:
               max1 = curmax

         elif plot.endswith('y'):
            if curmin < min2:
               min2 = curmin
            if curmax > max2:
               max2 = curmax

         else:
            if curmin < min3:
               min3 = curmin
            if curmax > max3:
               max3 = curmax

      margin = 0.2
      self._ax1.set_ylim( min1-margin, max1+margin )
      self._ax2.set_ylim( min2-margin, max2+margin )
      self._ax3.set_ylim( min3-margin, max3+margin )

      return returnVal


   def onresize(self, event):
      plt.tight_layout()
      #print 'Resized!'


class GraphPlot(object):
   def __init__(self):

      # Check if namespace is set.
      #try:
      #   ns = os.environ['ROS_NAMESPACE']
      #except:
      #   print 'ROS_NAMESPACE not set. Use "ns teamA/robotX" to set it.'
      #   exit()
      #if 'team' not in ns:
      #   print 'ROS_NAMESPACE not set. Use "ns teamA/robotX" to set it.'
      #   exit()

      lock = Lock()


      # Build the shared memory Array of type c_double that is used for plotting
      # _values is an array with for each plotted value the 'latest' value.
      dbl_list = [0.0] * len(_values_name)
      _values = Array(c_double, dbl_list, lock=lock)

      p = Process(target=myClass, args=(_values,_values_name))
      p.start()

      try:
         _graphs = LineGraph(_values, _values_name)
      except:
         exit()

      p.terminate()
      exit()

      print 'Redrawing graph at', 1.0/draw_freq, 'Hz'

      # Keep main thread running to redraw graph
      while True:
         before = int(round(time.time() * 1000))
         _graphs.update_graphs(_values)

         # Needed for redrawing graph (resizing)
         #QtGui.qApp.processEvents()
         after = int(round(time.time() * 1000))

         print 'redraw took', (after - before), 'milliseconds'

         # Sleep until after - before == draw_freq * 1000
         sleep_amount = float((draw_freq * 1000) - (after - before)) / 1000.0
         if sleep_amount > 0:
            time.sleep(sleep_amount)


class myClass(object):
   def __init__(self, values, values_name):

      self._values = values
      self._values_name = values_name
      self._previous_own_location = None

      prefix = "/teamA/robot" + ROBOT_NR + "/"

      # Init to ros as node with name 'plotter'
      rospy.init_node("plotter", anonymous=True)

      # subscribe to g_robotspeed
      # /teamA/robot1/g_robotspeed
      rospy.Subscriber(prefix + "g_robotspeed", t_robotspeed, self.cb_robotspeed)

      # subscribe to g_target
      # /teamA/robot1/g_target
      rospy.Subscriber(prefix + "g_target", t_target, self.cb_target)

      # subscribe to t_motor_robot_speed
      rospy.Subscriber(prefix + "t_motor_robot_speed", t_robotspeed, self.cb_motor_robotspeed)

      # subscribe to t_motor_robot_speed
      rospy.Subscriber(prefix + "t_motor_pid_params", t_motor_pid_params, self.cb_motor_pid_params)

      # Get own location service
      # Create a timer where, on each trigger, the s_get_own_location service is called.
      # /teamA/robot1/s_get_own_location
      rospy.wait_for_service(prefix + "s_get_own_location")
      self._get_own_location_srv = rospy.ServiceProxy(prefix + "s_get_own_location", get_own_location, True)
      rospy.Timer(rospy.Duration(_dt), self.cb_get_own_location)
      self.own_location = self._get_own_location_srv()

      # Keep the script alive
      rospy.spin()
      
   def cb_get_own_location(self, event):
      # Grab the location from the s_get_own_location service
      self.own_location = self._get_own_location_srv()
      
      # Prevent divide by zero on cold start
      if self._previous_own_location is None:
          self._previous_own_location = self.own_location
      
      if SOLX in self._values_name:
          self._values[self._values_name.index(SOLX)] = float(self.own_location.ownRobot.Pos.x)
          
      if SOLY in self._values_name:
          self._values[self._values_name.index(SOLY)] = float(self.own_location.ownRobot.Pos.y)

      if SOLTHETA in self._values_name:
          self._values[self._values_name.index(SOLTHETA)] = float(self.own_location.ownRobot.Pos.theta)

      if SOVX in self._values_name:
          self._values[self._values_name.index(SOVX)] = float(self.own_location.ownRobot.Vel.x)
          
      if SOVY in self._values_name:
          self._values[self._values_name.index(SOVY)] = float(self.own_location.ownRobot.Vel.y)

      if SOVTHETA in self._values_name:
          self._values[self._values_name.index(SOVTHETA)] = float(self.own_location.ownRobot.Vel.theta)
          
      if SOAX in self._values_name:
          self._values[self._values_name.index(SOAX)] = (self.own_location.ownRobot.Vel.x - self._previous_own_location.ownRobot.Vel.x) / _dt
          
      if SOAY in self._values_name:
          self._values[self._values_name.index(SOAY)] = (self.own_location.ownRobot.Vel.y - self._previous_own_location.ownRobot.Vel.y) / _dt

      if SOATHETA in self._values_name:
          self._values[self._values_name.index(SOATHETA)] = (self.own_location.ownRobot.Vel.theta - self._previous_own_location.ownRobot.Vel.theta) / _dt
          
      # Save velocity / location from previous iteration to compute acceleration
      self._previous_own_location = self.own_location

   def cb_robotspeed(self, data):
      # callback from g_robotspeed gives:
      #float32 vx
      #float32 vy
      #float32 vphi

      # For each value, check if we want to plot it.
      # If we want to plot it, set the value.

      if GRSVX in self._values_name:
         self._values[self._values_name.index(GRSVX)] = float(data.vx)

      if GRSVY in self._values_name:
         self._values[self._values_name.index(GRSVY)] = float(data.vy)

      if GRSVPHI in self._values_name:
         self._values[self._values_name.index(GRSVPHI)] = float(data.vphi)

   def cb_motor_robotspeed(self, data):
      # callback from t_motor_robot_speed gives:
      #float32 vx
      #float32 vy
      #float32 vphi

      # For each value, check if we want to plot it.
      # If we want to plot it, set the value.

      if GMRSVX in self._values_name:
         self._values[self._values_name.index(GMRSVX)] = float(data.vx)

      if GMRSVY in self._values_name:
         self._values[self._values_name.index(GMRSVY)] = float(data.vy)

      if GMRSVPHI in self._values_name:
         self._values[self._values_name.index(GMRSVPHI)] = float(data.vphi)

   def cb_motor_pid_params(self, data):
      # callback from t_motor_pid_params gives:
      #uint8 motorId
      #float32 setPoint
      #float32 measuredValue
      #float32 error
      #float32 integral
      #float32 derivative
      #float32 pidOutput
      #float32 displacement
      #float32 velocity
      #float32 velocityError

      vals={
              GMPPID: data.motorId,
              GMPPSP: data.setPoint,
              GMPPMV: data.measuredValue,
              GMPPERR: data.error,
              GMPPI: data.integral,
              GMPPD: data.derivative,
              GMPPPIDOUT: data.pidOutput,
              GMPPDISP: data.displacementEncTicks,
              GMPPVEL: data.velocity,
              GMPPVELERR: data.velocityError
       }

      # For each value, check if we want to plot it.
      # If we want to plot it, set the value.
    
      if data.motorId == 1:
         for val in vals.keys():
             if val in self._values_name:
                self._values[self._values_name.index(val)] = vals[val]

   def cb_target(self, data):

      # For each value, check if we want to plot it.
      # If we want to plot it, set the value.

      if GTPX in self._values_name:
         self._values[self._values_name.index(GTPX)] = float(data.x)

      if GTPY in self._values_name:
         self._values[self._values_name.index(GTPY)] = float(data.y)

      if GTPTHETA in self._values_name:
         self._values[self._values_name.index(GTPTHETA)] = float(data.phi)

      if GTVX in self._values_name:
         self._values[self._values_name.index(GTVX)] = float(data.vx)

      if GTVY in self._values_name:
         self._values[self._values_name.index(GTVY)] = float(data.vy)

      if GTVZ in self._values_name:
         self._values[self._values_name.index(GTVZ)] = float(data.vphi)

        
if __name__ == '__main__':

   bla = GraphPlot()
