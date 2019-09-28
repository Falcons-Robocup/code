""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # T1 = the phase where the acceleration is positive, gradually increasing the velocity to v_max
# T2 = the phase where the acceleration is 0, having a constant velocity of v_max
# T3 = the phase where the acceleration is negative, gradually decreasing the velocity to the target velocity
# at the end of T3, the target velocity and target position are reached

# 1. To find T2 we need T1 + T3.
# 1.1. Assume T2 > 0.
#
# T1 is the time to reach max velocity, given an initial velocity, as fast as possible (max acc).
# T1 = (v_max - v0) / a_max
# T3 = (v_max - v_end) / a_max
# 
# 1.2. Determine T2
#
# T2 is the cruising phase with acceleration = 0, velocity = v_max.
# The time spent at v_max is derived from the delta positions from T1 + T3.
# So first find the delta positions for these phases.
# 
# Delta position depends on the the initial velocity.
# Compute the delta velocity for each time period.
#
# deltaV_T1 = computeVelocity    ( T1, v_0, a_max )
# deltaS_T1 = computeDisplacement( T1, s_0, v_0, a_max )
#
# totalV_T1 = deltaV_T1
# totalS_T1 = deltaS_T1
#
# Initial position and velocity of each time period is the end position and velocity of the previous time period.
#
# T3 is after T2 which is the cruising phase with acceleration=0, velocity=v_max.
# So we can make some assumptions on the initial velocity of T3.
#
# deltaV_T3 = self.computeVelocity    ( T3, v_max, -a_max )         # T3 starts with vel=v_max
# deltaS_T3 = self.computeDisplacement( T3, 0.0, v_max, -a_max )    # T3 starts with vel=v_max
#
# totalV_T3 = self.computeVelocity    ( T3, v_max, -a_max )         # T3 starts with vel=v_max
#
# Now that we have the delta positions for all periods, compute the delta position for T2.
# deltaS_T2 = (desired_position - s_0) - (deltaS_T1 + deltaS_T3)
#
# T2's delta position runs at a constant velocity, compute T2.
# T2 = deltaS_T2 / v_max

import sys
import numpy as np
import subprocess
import itertools

    
def getFirstPositiveRealCandidate(values):
    result = 0.0
    real_values = values.real[abs(values.imag)<1e-5]
    
    positive_values = [positive_value for positive_value in real_values if positive_value >= 0]
    if len(positive_values) > 0:
        result = positive_values[0]
    else:
        print "Unable to find positive real candidate!"
    return result

class TrajectoryPlanner():

    def __init__(self):
        #self.max_jerk = np.array([5.0, 0.5, 0.5])
        #self.max_acceleration = np.array([5.0, 1.0, 1.0])
        #self.max_velocity = np.array([3.5, 5.0, 5.0])
        #self.epsilon = 0.001
        pass
        
    def plot(self, t, periods):
         acc = np.array([0.0])

         if t < periods[0]:
            acc[0] = self.amax * self.d
         elif t < (periods[0] + periods[1]):
            acc[0] = 0
         elif t < (periods[0] + periods[1] + periods[2]):
            acc[0] = -self.amax * self.d
                
         return acc

    def computeDisplacement(self, t, x0, v0, a):
      return ( x0 + (v0 * t) + (0.5 * a * (t**2)) )
      
    def computeVelocity(self, t, v0, a):
      return ( v0 + (a * t) )

    def computeTimePeriod(self, t, s_0, v_0, a_0, j):
        deltaA = self.computeAcceleration( t, 0.0, j ) 
        deltaV = self.computeVelocity    ( t, 0.0, a_0, j )
        deltaS = self.computeDisplacement( t, 0.0, v_0, a_0, j )

        totalA = self.computeAcceleration( t, a_0, j ) 
        totalV = self.computeVelocity    ( t, v_0, a_0, j )
        totalS = self.computeDisplacement( t, s_0, v_0, a_0, j )

        return (deltaS, deltaV, deltaA, totalS, totalV, totalA)
        
    def computeD(self, s_0, v_0, s_end, v_end):
        Serror = s_end - s_0
        d = np.sign(Serror)
        
        # Second on v_0. Third on Verror
        
        # second
        if d == 0.0:
            d = np.sign(-v_0)
                
            # third
            if d == 0.0:
                d = np.sign(v_end - v_0)
            
        return d

    # Overshoot? 
    def binarySearchVmaxOvershoot(self, s_0, v_0, s_end, v_end, v_max, a_max, T1, T2, T3):
        v_min = 0.0
        v = v_max
        idx = 0
        while abs(T2) > epsilon:
            # Find new T2
            
            last_v = v
            v = (v_max + v_min) / 2.0
            
            #print "new v_max", v
            T3 = (v - v_end) / a_max
            
            deltaS_T3 = self.computeDisplacement( T3, 0.0, v, -a_max )    # T3 starts with vel=v_max

            deltaS_T2 = (s_end - s_0) - (deltaS_T3)

            # T2's delta position runs at a constant velocity, compute T2.
            T2 = deltaS_T2 / v
            
            #print "iteration", idx, ": v_max =", v, ", T2 =", T2
            
            if T2 > epsilon:
               v_min = v
            elif T2 < epsilon:
               v_max = v
            else:
               last_v = v
               break
            
            # safety
            if idx > 30:
               break
            
            idx += 1

    # Triangle. T1 and T3 shall cover the full displacement.
    # Find a v_peak such that T1 + T3 reach the target setpoint
    def binarySearchVmax(self, s_0, v_0, s_end, v_end, v_max, a_max, T1, T2, T3):
        v_min = 0.0
        v = v_max
        idx = 0
        while abs(T2) > self.epsilon:
            # Find new T2
            
            last_v = v
            v = (v_max + v_min) / 2.0
            
            #print "new v_max", v
            T1  = (v - v_0) / a_max
            T3 = (v - v_end) / a_max
            
            deltaS_T1 = self.computeDisplacement( T1, 0.0, v_0, a_max )
            deltaS_T3 = self.computeDisplacement( T3, 0.0, v, -a_max )    # T3 starts with vel=v_max

            deltaS_T2 = (s_end - s_0) - (deltaS_T1 + deltaS_T3)

            # T2's delta position runs at a constant velocity, compute T2.
            T2 = deltaS_T2 / v
            
            #print "iteration", idx, ": v_max =", v, ", T2 =", T2
            
            if T2 > self.epsilon:
               v_min = v
            elif T2 < self.epsilon:
               v_max = v
            else:
               last_v = v
               break
            
            # safety
            if idx > 30:
               break
            
            idx += 1

        return last_v

    def computeTrajectory(self, s_0, v_0, s_end, v_end, v_max, a_max):
        

        ######### work starts here

        
        v_max = self.d * v_max
        a_max = self.d * a_max

        #target_pos = desired_position[0]
        
        epsilon = self.epsilon

        T1 = (v_max - v_0) / a_max
        T3 = (v_max - v_end) / a_max
        
        deltaS_T1 = self.computeDisplacement( T1, 0.0, v_0, a_max )
        deltaS_T3 = self.computeDisplacement( T3, 0.0, v_max, -a_max )    # T3 starts with vel=v_max

        deltaS_T2 = (s_end - s_0) - (self.d * (deltaS_T1 + deltaS_T3))
        T2 = deltaS_T2 / v_max

        print "s_0", s_0
        print "s_end", s_end

        print "deltaS_T1", deltaS_T1
        print "deltaS_T2", deltaS_T2
        print "deltaS_T3", deltaS_T3
        print "s_reached", (s_0 + deltaS_T1 + deltaS_T2 + deltaS_T3)
        print "self.d", self.d

        # Determine if we have overshoot when we climb to v_max
        if ((s_end - s_0) - (deltaS_T1 + deltaS_T2 + deltaS_T3)) * self.d < 0.0:
            print "Overshoot"

            # Case 1: T1 > 0
            # find new v_peak
            new_v_max = self.binarySearchVmax(s_0, v_0, s_end, v_end, v_max, a_max, T1, T2, T3)

            T1 = (new_v_max - v_0) / a_max
            T2 = 0.0
            T3 = (new_v_max - v_end) / a_max

            print "T1", T1
            print "T3", T3

            deltaS_T1 = self.computeDisplacement( T1, 0.0, v_0, a_max )
            deltaS_T3 = self.computeDisplacement( T3, 0.0, new_v_max, -a_max )    # T3 starts with vel=v_max
            print "deltaS_T1", deltaS_T1
            print "deltaS_T3", deltaS_T3
            print "s_reached Case1", (s_0 + deltaS_T1 + deltaS_T3)

            #print "Error", (s_end - s_0)
            #print "displacement", (deltaS_T1 + deltaS_T3)
            #print "Case 2 check", ((s_end - s_0) - (deltaS_T1 + deltaS_T3)) * self.d
            #if ((s_end - s_0) - (deltaS_T1 + deltaS_T3)) * self.d < self.epsilon:
            if T1 < 0.0:

                # Case 2: T1 = 0
                T1 = 0
                T2 = 0

                # Determine new v_peak
                T3 = v_0 / a_max
                print "s_reached Case3", (s_0 + deltaS_T3)
        else:

            # T2's delta position runs at a constant velocity, compute T2.
            
            if T2 < 0:
                print "T2 < 0"

                # T1 + T2 should cover the total displacement.
                # Start binary search for v_peak such that T2 = 0
                new_v_max = self.binarySearchVmax(s_0, v_0, s_end, v_end, v_max, a_max, T1, T2, T3)

                T1 = (new_v_max - v_0) / a_max
                T2 = 0.0
                T3 = (new_v_max - v_end) / a_max


        return [T1, T2, T3]

        ##### work ends here
        
        
    def computeFullTrajectory(self, s_0, v_0, s_end, v_end, v_max, a_max):
    
        self.d = self.computeD(s_0, v_0, s_end, v_end)

        epsilon = 0.0001
        self.epsilon = epsilon
        
        # If computeD returns 0, there is no error in S, V or A. So all times can be 0.
        if self.d == 0.0:
            return [0.0, 0.0, 0.0]
        
        self.amax = a_max

        result = -1


        result = self.computeTrajectory(s_0, v_0, s_end, v_end, v_max, a_max)

        return result
        
        
if __name__ == "__main__":

   t = TrajectoryPlanner()
   
   #s_0 = 0.0
   #v_0 = 0.5
   #a_0 = -1.0
   #
   #s_end = 3.5
   #v_end = -1.0
   #a_end = 0.0
   #
   #v_max = 3.5
   #a_max = 5.0
   #j_max = 5.0
   
   s_0lst = np.arange(-1.0, 1.0, 0.5) #4
   v_0lst = np.arange(-1.0, 1.0, 0.5) #4
   a_0lst = np.arange(-1.0, 1.0, 0.5) #4
   
   s_endlst = np.arange(-10.0, 10.0, 2.0) #10
   v_endlst = np.arange(-1.0, 1.0, 0.5) #4
   #a_endlst = np.arange(-1.0, 1.0, 0.5) #4
   a_endlst = [0.0]
   
   v_maxlst = np.arange(1.0, 5.0, 1.0) #5
   a_maxlst = np.arange(1.0, 5.0, 1.0) #5
   j_maxlst = np.arange(1.0, 5.0, 1.0) #5
   
   total_nr_combinations = len(s_0lst) * len(v_0lst) * len(a_0lst) * len(s_endlst) * len(v_endlst) * len(a_endlst) * len(v_maxlst) * len(a_maxlst) * len(j_maxlst)
   
   idx = 0
   
   for s_0, v_0, a_0, s_end, v_end, a_end, v_max, a_max, j_max in itertools.product(s_0lst, v_0lst, a_0lst, s_endlst, v_endlst, a_endlst, v_maxlst, a_maxlst, j_maxlst):
      
      result = t.computeFullTrajectory(s_0, v_0, a_0, s_end, v_end, a_end, v_max, a_max, j_max)
      
      conclusion = "OK"
      
      d = t.computeD(s_0, v_0, a_0, s_end, v_end, a_end)
      
      if result == -1:
        print "Not supported -- skipping."
      else:
      
          if len(result) > 7:
            # preperiod included
            [pT1, pT2, pT3, T1, T2, T3, T4, T5, T6, T7] = result
            
            #print [pT1, pT2, pT3, T1, T2, T3, T4, T5, T6, T7]
            
            pT1_vals = t.computeTimePeriod(pT1, s_0, v_0, a_0, j_max*-d)
            pT2_vals = t.computeTimePeriod(pT2, pT1_vals[3], pT1_vals[4], pT1_vals[5], 0.0)
            pT3_vals = t.computeTimePeriod(pT3, pT2_vals[3], pT2_vals[4], pT2_vals[5], -j_max*-d)
            T1_vals = t.computeTimePeriod(T1, pT3_vals[3], pT3_vals[4], pT3_vals[5], j_max*d)
            T2_vals = t.computeTimePeriod(T2, T1_vals[3], T1_vals[4], T1_vals[5], 0.0)
            T3_vals = t.computeTimePeriod(T3, T2_vals[3], T2_vals[4], T2_vals[5], -j_max*d)
            T4_vals = t.computeTimePeriod(T4, T3_vals[3], T3_vals[4], T3_vals[5], 0.0)
            T5_vals = t.computeTimePeriod(T5, T4_vals[3], T4_vals[4], T4_vals[5], -j_max*d)
            T6_vals = t.computeTimePeriod(T6, T5_vals[3], T5_vals[4], T5_vals[5], 0.0)
            T7_vals = t.computeTimePeriod(T7, T6_vals[3], T6_vals[4], T6_vals[5], j_max*d)
            
            print "preperiod reached position:", pT3_vals[3]
            #print "preperiod reached velocity:", pT3_vals[4]
            #print "preperiod reached acceleration:", pT3_vals[5]

            
            
          else:
          
            [T1, T2, T3, T4, T5, T6, T7] = result
          
            T1_vals = t.computeTimePeriod(T1, s_0, v_0, a_0, j_max*d)
            T2_vals = t.computeTimePeriod(T2, T1_vals[3], T1_vals[4], T1_vals[5], 0.0)
            T3_vals = t.computeTimePeriod(T3, T2_vals[3], T2_vals[4], T2_vals[5], -j_max*d)
            T4_vals = t.computeTimePeriod(T4, T3_vals[3], T3_vals[4], T3_vals[5], 0.0)
            T5_vals = t.computeTimePeriod(T5, T4_vals[3], T4_vals[4], T4_vals[5], -j_max*d)
            T6_vals = t.computeTimePeriod(T6, T5_vals[3], T5_vals[4], T5_vals[5], 0.0)
            T7_vals = t.computeTimePeriod(T7, T6_vals[3], T6_vals[4], T6_vals[5], j_max*d)

          
          
          actual_s_end = T7_vals[3]
          actual_v_end = T7_vals[4]
          actual_a_end = T7_vals[5] * -1 # Acceleration always comes out the other way
          
          epsilon = 0.01
          
          pos_ok = ( abs(actual_s_end - s_end) < epsilon )
          vel_ok = ( abs(actual_v_end - v_end) < epsilon )
          acc_ok = ( abs(actual_a_end - a_end) < epsilon )

          if len(result) > 7: #preperiod present
              # accept overshoot from preperiod
              if d < 0:
                  if pT3_vals[3] < s_end:
                      print "Preperiod overshoots target. Accepted."
                      pos_ok = True
                      vel_ok = True
              else:
                  if pT3_vals[3] > s_end:
                      print "Preperiod overshoots target. Accepted."
                      pos_ok = True
                      vel_ok = True

          if not pos_ok or not vel_ok or not acc_ok:
             conclusion = "NOK"
             print "The following parameters failed:"
             print "s_0:", s_0
             print "v_0:", v_0
             print "a_0:", a_0
             print "s_end:", s_end
             print "v_end:", v_end
             print "a_end:", a_end
             print "v_max:", v_max
             print "a_max:", a_max
             print "j_max:", j_max
             print "periods:", result, "(total:", str(sum(result)), ")"
          
          print "pos: %s -- %s (%s)" % ( "OK" if pos_ok else "NOK", str(actual_s_end), str(s_end) )
          print "vel: %s -- %s (%s)" % ( "OK" if vel_ok else "NOK", str(actual_v_end), str(v_end) )
          print "acc: %s -- %s (%s)" % ( "OK" if acc_ok else "NOK", str(actual_a_end), str(a_end) )
          
          
      print "Conclusion:", conclusion
      print "-------------------------------------"
          
      #if idx > 50000:
      #  exit()
      
      if idx % 10000 == 0:
        print "======================================"
        print "======================================"
        print "======================================"
        print "Processed %s out of %s trajectories." % ( str(idx), str(total_nr_combinations) )
        print "======================================"
        print "======================================"
        print "======================================"
      
      idx += 1
