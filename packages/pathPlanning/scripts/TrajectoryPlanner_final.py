""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # T1 = the phase where the jerk is positive, gradually increasing the acceleration to a_max 
# T2 = the phase where the jerk is 0, having a constant acceleration of a_max
# T3 = the phase where the jerk is negative, gradually decreasing the acceleration to 0
# at the end of T3, the maximum velocity is reached
# T4 = the phase where the jerk is 0, acceleration is 0, and velocity is constant at v_max.
# T5 = the phase where the jerk is negative, gradually increasing the deceleration to -a_max.
# T6 = the phase where the jerk is 0, having a constant deceleration of -a_max.
# T7 = the phase where the jerk is positive, gradually decreasing the deceleration to the target acceleration
# at the end of T7, the target velocity and target position are reached.

# If v_max is not reached, T4 = 0.
# If a_max is not reached in the acceleration phase, T2 = 0.
# If -a_max is not reached in the deceleration phase, T6 = 0.

# 1. To find T4, we need T1-T3 + T5-T7.
# 1.1 We first compute T1, T2 and T3.
# 1.1.1 Assume T2 > 0.
#
# T1 is the time to reach max acceleration, given an initial acceleration, as fast as possible (max jerk).
# T1 = (a_max - a0) / j_max
#
# T3 is the time to decrease acceleration from the max acceleration.
# T3 = (a_max / j_max)
#
# T2 is the time spent at maximum acceleration to reach the maximum velocity when T3 has completed.
# T2 is derived based on the velocity increase from T1 and T3.
# 
# T1's delta velocity is computed using the initial velocity, initial acceleration and maximum jerk.
# deltaV_T1 = computeVelocity( T1, v0, a0, j_max )
#
# T3's delta velocity is computed by decelerating from a_max to 0.0 as fast as possible (max jerk).
# deltaV_T3 = computeVelocity( T3, 0.0, a_max, -j_max )
#
# T2's delta velocity fills the gap of reaching the maximum velocity
# deltaV_T2 = v_max - deltaV_T1 - deltaV_T3
# 
# Since T2 is by definition at max acceleration, compute the time to reach deltaV_T2
# T2 = deltaV_T2 / a_max
#
# 1.1.2 If T2 < 0.
#
# It is possible that during the acceleration phase (T1-T3), a_max is not reached.
# This means T2 < 0.
# To ensure it is not used:
# T2 = 0
# 
# Because initial acceleration (a_0) can be > 0, T1 and T3 may not be symmetrical.
# Define T0 as the phase before T1 where the accelerating increases from 0 to a_0.
# This makes T0 + T1 symmetrical to T3.
# T0 = a_0 / j_max
#
# To find the initial velocity before T0, find delta velocity of T0.
# deltaV_T0 = computeVelocity( T0, 0.0, 0.0, j_max )
#
# Find initial velocity before T0
# velocity_T0_begin = v_0 - deltaV_T0
# 
# T0 + T1 is symmetrical to T3.
# The delta velocity from velocity_T0_begin to v_max is done in T0 to T3.
# deltaV_T0_T1_T3 = v_max - velocity_T0_begin
#
# Half of the delta velocity is done by T3, the other half by T0 + T1.
# deltaV_T3 = deltaV_T0_T1_T3 / 2.0
#
# Having the delta velocity handled by T3, we can get the delta velocity handled by T1.
# deltaV_T1 = v_max - v_0 - deltaV_T3
#
# Now we have the delta velocities for T1 and T3.
# To find T1, we should solve the following equation:
#
## deltaV_T1 = (a_0 * T1) + (0.5 * j_max * (T1**2))
## =>
## -deltaV_T1 + (a_0 * T1) + (0.5 * j_max * (T1**2)) = 0
#
# coeff = [(0.5*j_max), a_0, -(deltaV_T1)]
# T1_candidates = np.roots(coeff)
# T1 = getFirstPositiveRealCandidate(T1_candidates)
#
# Now that we have T1, find the maximum reached acceleration
# maxA_T1 = (j_max * T1) + a_0
#
# Having the maximum reached acceleration, T3 can decrease the acceleration to 0 as fast as possible (max_jerk)
# T3 = maxA_T1 / j_max
#
# 1.2 We now compute T5, T6 and T7.
# This happens in a similar manner as T1, T2 and T3.
#
# 2. Having T1-T3 + T5-T7, find T4.
# 
# T4 is the cruising phase with jerk = 0, acceleration = 0, velocity = v_max.
# The time spent at v_max is derived from the delta positions from T1-to-T3 + T5-to-T7.
# So first find the delta positions for these phases.
# 
# Delta position depends on the the initial velocity and acceleration.
# Compute the delta velocity and acceleration for each time period.
#
# deltaA_T1 = computeAcceleration( T1, a_0, j_max )
# deltaV_T1 = computeVelocity    ( T1, v_0, a_0, j_max )
# deltaS_T1 = computeDisplacement( T1, s_0, v_0, a_0, j_max )
#
# totalA_T1 = deltaA_T1
# totalV_T1 = deltaV_T1
# totalS_T1 = deltaS_T1
#
# Initial position, velocity and acceleration of each time period is the end position, velocity and acceleration of the previous time period.
#
# jerk in T2 is 0.
# deltaA_T2 = self.computeAcceleration( T2, 0.0, 0.0 ) 
# deltaV_T2 = self.computeVelocity    ( T2, 0.0, totalA_T1, 0.0 )
# deltaS_T2 = self.computeDisplacement( T2, 0.0, totalV_T1, totalA_T1, 0.0 )
#
# totalA_T2 = self.computeAcceleration( T2, totalA_T1, 0.0 ) 
# totalV_T2 = self.computeVelocity    ( T2, totalV_T1, totalA_T1, 0.0 )
# totalS_T2 = self.computeDisplacement( T2, totalS_T1, totalV_T1, totalA_T1, 0.0 )
#
#
# jerk in T3 is negative.
# deltaA_T3 = self.computeAcceleration( T3, 0.0, -j_max) 
# deltaV_T3 = self.computeVelocity    ( T3, 0.0, (deltaA_T1 + deltaA_T2), -j_max )
# deltaS_T3 = self.computeDisplacement( T3, 0.0, (deltaV_T1 + deltaV_T2), (deltaA_T1 + deltaA_T2), -j_max )
#
# totalA_T3 = self.computeAcceleration( T3, totalA_T2, -j_max) 
# totalV_T3 = self.computeVelocity    ( T3, totalV_T2, totalA_T2, -j_max )
# totalS_T3 = self.computeDisplacement( T3, totalS_T2, totalV_T2, totalA_T2, -j_max )
#
#
# T5 is after T4 which is the cruising phase with jerk=0, acceleration=0, velocity=v_max.
# So we can make some assumptions on the initial acceleration and velocity of T5.
# deltaA_T5 = self.computeAcceleration( T5, 0.0, -j_max)                 # T5 starts with acc=0
# deltaV_T5 = self.computeVelocity    ( T5, v_max, 0.0, -j_max )         # T5 starts with vel=v_max, acc=0
# deltaS_T5 = self.computeDisplacement( T5, 0.0, v_max, 0.0, -j_max )    # T5 starts with vel=v_max, acc=0
#
# totalA_T5 = self.computeAcceleration( T5, 0.0, -j_max)                 # T5 starts with acc=0
# totalV_T5 = self.computeVelocity    ( T5, v_max, 0.0, -j_max )         # T5 starts with vel=v_max, acc=0
#
#
# jerk in T6 is 0.
# deltaA_T6 = self.computeAcceleration( T6, 0.0, 0.0 ) 
# deltaV_T6 = self.computeVelocity    ( T6, 0.0, deltaA_T5, 0.0 )
# deltaS_T6 = self.computeDisplacement( T6, 0.0, deltaV_T5, deltaA_T5, 0.0 )
#
# totalA_T6 = self.computeAcceleration( T6, totalA_T5, 0.0 ) 
# totalV_T6 = self.computeVelocity    ( T6, totalV_T5, totalA_T5, 0.0 )
#
#
# jerk in T7 is positive.
# deltaA_T7 = self.computeAcceleration( T7, 0.0, j_max) 
# deltaV_T7 = self.computeVelocity    ( T7, 0.0, totalA_T6, j_max )
# deltaS_T7 = self.computeDisplacement( T7, 0.0, totalV_T6, totalA_T6, j_max )
#
# totalA_T7 = self.computeAcceleration( T7, totalA_T6, j_max) 
# totalV_T7 = self.computeVelocity    ( T7, totalV_T6, totalA_T6, j_max )
#
#
# Now that we have the delta positions for all periods, compute the delta position for T4.
# deltaS_T4 = (desired_position - s_0) - (deltaS_T1 + deltaS_T2 + deltaS_T3 + deltaS_T5 + deltaS_T6 + deltaS_T7)
#
# T4's delta position runs at a constant velocity, compute T4.
# T4 = deltaS_T4 / v_max

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
         jerk = np.array([0.0])
         
         if len(periods) > 7:
         
             if t < periods[0]:
                jerk[0] = self.jmax * self.preperiodD
             elif t < (periods[0] + periods[1]):
                jerk[0] = 0
             elif t < (periods[0] + periods[1] + periods[2]):
                jerk[0] = -self.jmax * self.preperiodD
             elif t < (periods[0] + periods[1] + periods[2] + periods[3]):
                jerk[0] = self.jmax * self.d
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4]):
                jerk[0] = 0
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4] + periods[5]):
                jerk[0] = -self.jmax * self.d
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4] + periods[5] + periods[6]):
                jerk[0] = 0
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4] + periods[5] + periods[6] + periods[7]):
                jerk[0] = -self.jmax * self.d
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4] + periods[5] + periods[6] + periods[7] + periods[8]):
                jerk[0] = 0
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4] + periods[5] + periods[6] + periods[7] + periods[8] + periods[9]):
                jerk[0] = self.jmax * self.d
             else:
                jerk[0] = 0
                
         else:
             if t < periods[0]:
                jerk[0] = self.jmax * self.d
             elif t < (periods[0] + periods[1]):
                jerk[0] = 0
             elif t < (periods[0] + periods[1] + periods[2]):
                jerk[0] = -self.jmax * self.d
             elif t < (periods[0] + periods[1] + periods[2] + periods[3]):
                jerk[0] = 0
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4]):
                jerk[0] = -self.jmax * self.d
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4] + periods[5]):
                jerk[0] = 0
             elif t < (periods[0] + periods[1] + periods[2] + periods[3] + periods[4] + periods[5] + periods[6]):
                jerk[0] = self.jmax * self.d
             else:
                jerk[0] = 0
                
         return jerk

    def computeDisplacement(self, t, x0, v0, a0, j):
      return ( x0 + (v0 * t) + (0.5 * a0 * (t**2)) + ((1.0/6.0) * j * (t**3)) )
      
    def computeVelocity(self, t, v0, a0, j):
      return ( v0 + (a0 * t) + (0.5 * j * (t**2)) )
      
    def computeAcceleration(self, t, a0, j):
      return ( a0 + (j * t) )

    def computeT1T2T3(self, v_0, a_0, v_max, a_max, j_max):

        #print "a_max", a_max
        #print "v_max", v_max
        T1 = (a_max - a_0) / j_max
        #print "T1 to reach a_max from a_0", T1
        T3 = (a_max / j_max)
        #print "T3 to reach a=0", T3
        #
        # T2 is the time spent at maximum acceleration to reach the maximum velocity when T3 has completed.
        # T2 is derived based on the velocity increase from T1 and T3.
        # 
        # T1's delta velocity is computed using the initial velocity, initial acceleration and maximum jerk.
        deltaV_T1 = self.computeVelocity( T1, v_0, a_0, j_max )
        #
        # T3's delta velocity is computed by decelerating from a_max to 0.0 as fast as possible (max jerk).
        deltaV_T3 = self.computeVelocity( T3, 0.0, a_max, -j_max )
        #
        # T2's delta velocity fills the gap of reaching the maximum velocity
        deltaV_T2 = v_max - deltaV_T1 - deltaV_T3
        #print "deltaV_T2", deltaV_T2
        # 
        # Since T2 is by definition at max acceleration, compute the time to reach deltaV_T2
        T2 = deltaV_T2 / a_max
        
        if T2 < 0.0:
            #print "T2 < 0"

            # It is possible that during the acceleration phase (T1-T3), a_max is not reached.
            # This means T2 < 0.
            # To ensure it is not used:
            T2 = 0
            #print "T2 < 0"
            
            # if a_0 != 0, we should compute a T0.

            # different case for a_0 < 0
            if a_0 < 0:

                # Because a_0 is negative, T1 is split into two periods: T0 and T1.
                # In T0+T1 the acceleration climbs from a_0 to a_max.
                # In T0 the acceleration climbs from a_0 to 0.
                # In T1 the acceleration climbs from 0 to a_max.

                # Compute the time for T0
                T0 = abs(a_0) / j_max
                #print "T0", T0

                # Compute the time for T1
                T1 = a_max / j_max
                #print "T1", T1

            else:
            
                # Because initial acceleration (a_0) can be > 0, T1 and T3 may not be symmetrical.
                # Define T0 as the phase before T1 where the accelerating increases from 0 to a_0.
                # This makes T0 + T1 symmetrical to T3.
                T0 = a_0 / j_max
                #print "T0", T0

            # To find the initial velocity before T0, find delta velocity of T0.
            deltaV_T0 = self.computeVelocity( T0, 0.0, 0.0, j_max )

            # Find initial velocity before T0
            velocity_T0_begin = v_0 - deltaV_T0
            #print "velocity_T0_begin", velocity_T0_begin

            # T0 + T1 is symmetrical to T3.
            # The delta velocity from velocity_T0_begin to v_max is done in T0 to T3.
            deltaV_T0_T1_T3 = v_max - velocity_T0_begin
            #print "deltaV_T0_T1_T3", deltaV_T0_T1_T3

            # Half of the delta velocity is done by T3, the other half by T0 + T1.
            deltaV_T3 = deltaV_T0_T1_T3 / 2.0

            # Having the delta velocity handled by T3, we can get the delta velocity handled by T1.
            deltaV_T1 = v_max - v_0 - deltaV_T3
            #print "deltaV_T1", deltaV_T1
            #print "a_0", a_0
            #print "j_max", j_max
            
            if (self.d * deltaV_T1) < -self.epsilon:
                #print "WARNING: No solution can be found for T1."
                #return 0.0, 0.0, 0.0
                # Assuming T1 is 0.

                # deltaV is only for T2 and T3.
                # deltaV_T1 is done by T2.
                #T2 = deltaV_T1 / a_0
                # Only T3 remaining.
                T3 = a_0 / j_max

                return 0.0, 0.0, T3

            # Now we have the delta velocities for T1 and T3.
            # To find T1, we should solve the following equation:
            #
            ## deltaV_T1 = (a_0 * T1) + (0.5 * j_max * (T1**2))
            ## =>
            ## -deltaV_T1 + (a_0 * T1) + (0.5 * j_max * (T1**2)) = 0
            #
            coeff = [(0.5*j_max), a_0, -(deltaV_T1)]
            T1_candidates = np.roots(coeff)
            T1 = getFirstPositiveRealCandidate(T1_candidates)
            #
            # Now that we have T1, find the maximum reached acceleration
            maxA_T1 = (j_max * T1) + a_0
            #
            # Having the maximum reached acceleration, T3 can decrease the acceleration to 0 as fast as possible (max_jerk)
            T3 = maxA_T1 / j_max



        return T1, T2, T3

    def computeT5T6T7(self, a_0, v_end, a_end, v_max, a_max, j_max):
        T7 = (a_max - a_end) / j_max
        T5 = (a_max / j_max)
        #
        deltaV_T7 = self.computeVelocity( T7, v_end, a_end, j_max )
        #
        deltaV_T5 = self.computeVelocity( T5, 0.0, a_max, -j_max )
        #
        deltaV_T6 = v_max - deltaV_T5 - deltaV_T7
        # 
        T6 = deltaV_T6 / a_max
        
        if T6 < 0:
            #print "T6 < 0"

            # To ensure it is not used:
            T6 = 0
            
            # different case for a_0 < 0
            if a_0 < 0:

                # Because a_0 is negative, T7 is split into two periods: T7 and T8.
                # In T7+T8 the deceleration decreases from a_max to a_end.
                # In T7 the deceleration decreaes from a_max to 0.
                # In T8 the deceleration decreases from 0 to a_end.

                # Compute the time for T7
                #print "T7 before", T7
                T7 = a_max / j_max
                #print "T7", T7

                # Compute the time for T8
                T8 = a_end / j_max
                #print "T8", T8

            else:
            
                
                # 
                # Because initial acceleration (a_0) can be > 0, T1 and T3 may not be symmetrical.
                # Define T0 as the phase before T1 where the accelerating increases from 0 to a_0.
                # This makes T0 + T1 symmetrical to T3.
                T8 = a_end / j_max
                #print "T8", T8
            

            # To find the initial velocity before T0, find delta velocity of T0.
            deltaV_T8 = self.computeVelocity( T8, 0.0, 0.0, j_max )
            #print "deltaV_T8", deltaV_T8
            #
            # Find end velocity after T8
            velocity_T8_end = v_end - deltaV_T8
            #print "velocity_T8_end", velocity_T8_end
            # 
            deltaV_T5_T7_T8 = v_max - velocity_T8_end
            #print "deltaV_T5_T7_T8", deltaV_T5_T7_T8
            #
            deltaV_T5 = deltaV_T5_T7_T8 / 2.0
            #print "deltaV_T5", deltaV_T5
            #
            deltaV_T7 = v_max - v_end - deltaV_T5
            #print "deltaV_T7", deltaV_T7
            
            if (self.d * deltaV_T7) < 0.0:
                #print "WARNING: No solution can be found for T7."
                T7 = a_end / j_max

                return 0.0, 0.0, T7
            
            # Now we have the delta velocities for T1 and T3.
            # To find T1, we should solve the following equation:
            #
            ## deltaV_T1 = (a_0 * T1) + (0.5 * j_max * (T1**2))
            ## =>
            ## -deltaV_T1 + (a_0 * T1) + (0.5 * j_max * (T1**2)) = 0
            #
            coeff = [(0.5*j_max), a_end, -(deltaV_T7)]
            T7_candidates = np.roots(coeff)
            T7 = getFirstPositiveRealCandidate(T7_candidates)
            #print "T7_candidates:", T7_candidates
            #
            # Now that we have T1, find the maximum reached acceleration
            maxA_T7 = (j_max * T7) + a_end
            #
            # Having the maximum reached acceleration, T3 can decrease the acceleration to 0 as fast as possible (max_jerk)
            T5 = maxA_T7 / j_max

        return T5, T6, T7

    def computeT4(self, T1, T2, T3, T5, T6, T7, s_0, v_0, a_0, v_max, a_max, j_max, desired_position):
        deltaA_T1 = self.computeAcceleration( T1, a_0, j_max )
        deltaV_T1 = self.computeVelocity    ( T1, v_0, a_0, j_max )
        deltaS_T1 = self.computeDisplacement( T1, 0.0, v_0, a_0, j_max )

        totalA_T1 = deltaA_T1
        totalV_T1 = deltaV_T1
        totalS_T1 = deltaS_T1


        # Initial position, velocity and acceleration of each time period is the end position, velocity and acceleration of the previous time period.
        #
        # jerk in T2 is 0.
        deltaA_T2 = self.computeAcceleration( T2, 0.0, 0.0 ) 
        deltaV_T2 = self.computeVelocity    ( T2, 0.0, totalA_T1, 0.0 )
        deltaS_T2 = self.computeDisplacement( T2, 0.0, totalV_T1, totalA_T1, 0.0 )

        totalA_T2 = self.computeAcceleration( T2, totalA_T1, 0.0 ) 
        totalV_T2 = self.computeVelocity    ( T2, totalV_T1, totalA_T1, 0.0 )
        totalS_T2 = self.computeDisplacement( T2, totalS_T1, totalV_T1, totalA_T1, 0.0 )

        # jerk in T3 is negative.
        deltaA_T3 = self.computeAcceleration( T3, 0.0, -j_max) 
        deltaV_T3 = self.computeVelocity    ( T3, 0.0, totalA_T2, -j_max )
        deltaS_T3 = self.computeDisplacement( T3, 0.0, totalV_T2, totalA_T2, -j_max )

        totalA_T3 = self.computeAcceleration( T3, totalA_T2, -j_max) 
        totalV_T3 = self.computeVelocity    ( T3, totalV_T2, totalA_T2, -j_max )
        totalS_T3 = self.computeDisplacement( T3, totalS_T2, totalV_T2, totalA_T2, -j_max )

        # T5 is after T4 which is the cruising phase with jerk=0, acceleration=0, velocity=v_max.
        # So we can make some assumptions on the initial acceleration and velocity of T5.
        deltaA_T5 = self.computeAcceleration( T5, 0.0, -j_max)                 # T5 starts with acc=0
        deltaV_T5 = self.computeVelocity    ( T5, v_max, 0.0, -j_max )         # T5 starts with vel=v_max, acc=0
        deltaS_T5 = self.computeDisplacement( T5, 0.0, v_max, 0.0, -j_max )    # T5 starts with vel=v_max, acc=0

        totalA_T5 = self.computeAcceleration( T5, 0.0, -j_max)                 # T5 starts with acc=0
        totalV_T5 = self.computeVelocity    ( T5, v_max, 0.0, -j_max )         # T5 starts with vel=v_max, acc=0

        # jerk in T6 is 0.
        deltaA_T6 = self.computeAcceleration( T6, 0.0, 0.0 ) 
        deltaV_T6 = self.computeVelocity    ( T6, 0.0, deltaA_T5, 0.0 )
        deltaS_T6 = self.computeDisplacement( T6, 0.0, deltaV_T5, deltaA_T5, 0.0 )

        totalA_T6 = self.computeAcceleration( T6, totalA_T5, 0.0 ) 
        totalV_T6 = self.computeVelocity    ( T6, totalV_T5, totalA_T5, 0.0 )

        # jerk in T7 is positive.
        deltaA_T7 = self.computeAcceleration( T7, 0.0, j_max) 
        deltaV_T7 = self.computeVelocity    ( T7, 0.0, totalA_T6, j_max )
        deltaS_T7 = self.computeDisplacement( T7, 0.0, totalV_T6, totalA_T6, j_max )

        totalA_T7 = self.computeAcceleration( T7, totalA_T6, j_max) 
        totalV_T7 = self.computeVelocity    ( T7, totalV_T6, totalA_T6, j_max )

        # Now that we have the delta positions for all periods, compute the delta position for T4.
        #print "(desired_position - s_0)", (desired_position - s_0)
        #print "(deltaS_T1 + deltaS_T2 + deltaS_T3 + deltaS_T5 + deltaS_T6 + deltaS_T7)", (deltaS_T1 + deltaS_T2 + deltaS_T3 + deltaS_T5 + deltaS_T6 + deltaS_T7)
        deltaS_T4 = (desired_position - s_0) - (deltaS_T1 + deltaS_T2 + deltaS_T3 + deltaS_T5 + deltaS_T6 + deltaS_T7)
        #print "deltaS_T4", deltaS_T4

        # T4's delta position runs at a constant velocity, compute T4.
        T4 = deltaS_T4 / v_max

        return T4

    def computeTimePeriod(self, t, s_0, v_0, a_0, j):
        deltaA = self.computeAcceleration( t, 0.0, j ) 
        deltaV = self.computeVelocity    ( t, 0.0, a_0, j )
        deltaS = self.computeDisplacement( t, 0.0, v_0, a_0, j )

        totalA = self.computeAcceleration( t, a_0, j ) 
        totalV = self.computeVelocity    ( t, v_0, a_0, j )
        totalS = self.computeDisplacement( t, s_0, v_0, a_0, j )

        return (deltaS, deltaV, deltaA, totalS, totalV, totalA)
        
    def computeD(self, s_0, v_0, a_0, s_end, v_end, a_end):
        Serror = s_end - s_0
        d = np.sign(Serror)
        
        # Second on v_0. Third on a_0. Fourth on Verror. Fifth on Aerror.
        
        # second
        if d == 0.0:
            d = np.sign(-v_0)
            
            # third
            if d == 0.0:
                d = np.sign(-a_0)
                
                # fourth
                if d == 0.0:
                    d = np.sign(v_end - v_0)
                    
                    # fifth
                    if d == 0.0:
                        d = np.sign(a_end - a_0)
            
        return d

    def computeTrajectory(self, s_0, v_0, a_0, s_end, v_end, a_end, v_max, a_max, j_max):
        
        #v_end = -1.0
        #a_end = 2.5
        
        #a_vector = x_vector[0:3]
        #v_vector = x_vector[3:6]
        #s_vector = x_vector[6:9]

        #s_error = desired_position - s_vector
        
        #d = np.sign(s_error[0])

        

        ######### work starts here

        #s_0 = s_vector[0]
        #v_0 = v_vector[0]
        #a_0 = a_vector[0]
        #v_max = self.max_velocity[0]
        #a_max = self.max_acceleration[0]
        #j_max = self.max_jerk[0]

        # Used by the plotter
        
        #d = self.computeD(s_0, v_0, a_0, s_end, v_end, a_end)
        
        v_max = self.d * v_max
        a_max = self.d * a_max
        j_max = self.d * j_max

        #target_pos = desired_position[0]
        
        epsilon = self.epsilon

        T1, T2, T3 = self.computeT1T2T3(v_0, a_0, v_max, a_max, j_max)
        
        #if T1 < 0: # and T2<0 and T3<0
        #    # We were unable to find a solution.
        #    return -1
        
        T5, T6, T7 = self.computeT5T6T7(a_0, v_end, a_end, v_max, a_max, j_max)
        
        if T7 < 0: # and T5<0 and T6<0
            # We were unable to find a solution.
            return -2

        T4 = self.computeT4(T1, T2, T3, T5, T6, T7, s_0, v_0, a_0, v_max, a_max, j_max, s_end)

        if T4 < 0:
           #print "T4 < 0"
           #T4 = 0
           
           #print T1, T2, T3, T4, T5, T6, T7
           
           # Start binary search for v_peak such that T4 = 0
           v_min = 0.0
           v = v_max
           idx = 0
           while abs(T4) > epsilon:
               # Find new T4
               
               last_v = v
               v = (v_max + v_min) / 2.0
               
               #print "new v_max", v
               T1, T2, T3 = self.computeT1T2T3(v_0, a_0, v, a_max, j_max)
               #print T1, T2, T3, T4, T5, T6, T7
               
               #if T1 < 0: # and T2<0 and T3<0
               #   # We were unable to find a solution.
               #   return -1

               # It is possible that T1=0, T2=0 and decreasing acceleration to 0 overshoots v.
               # For this scenario, we will overrule v_max with the new v_peak.
               T1_vals = self.computeTimePeriod(T1, s_0, v_0, a_0, j_max)
               T2_vals = self.computeTimePeriod(T2, T1_vals[3], T1_vals[4], T1_vals[5], 0.0)
               T3_vals = self.computeTimePeriod(T3, T2_vals[3], T2_vals[4], T2_vals[5], -j_max)

               v_peak = T3_vals[4]
               if self.d < 0:
                   if v_peak < -v:
                       #print "decreasing acceleration to 0 overshoots v"
                       T5, T6, T7 = self.computeT5T6T7(a_0, v_end, a_end, v_peak, a_max, j_max)
                       T4 = self.computeT4(T1, T2, T3, T5, T6, T7, s_0, v_0, a_0, v_peak, a_max, j_max, s_end)
                       if T4 < 0.0:
                          T4 = 0.0
                       break
                       #T3 = 0.0
               else:
                   if v_peak > v:
                       #print "decreasing acceleration to 0 overshoots v"
                       T5, T6, T7 = self.computeT5T6T7(a_0, v_end, a_end, v_peak, a_max, j_max)
                       T4 = self.computeT4(T1, T2, T3, T5, T6, T7, s_0, v_0, a_0, v_peak, a_max, j_max, s_end)
                       if T4 < 0.0:
                          T4 = 0.0
                       break
                       #T3 = 0.0
               
               T5, T6, T7 = self.computeT5T6T7(a_0, v_end, a_end, v, a_max, j_max)

               #print T1, T2, T3, T4, T5, T6, T7
               
               #if T7 < 0: # and T5<0 and T6<0
               #   print "T7 < 0"
               #   # We were unable to find a solution.
               #   return -2
               
               T4 = self.computeT4(T1, T2, T3, T5, T6, T7, s_0, v_0, a_0, v, a_max, j_max, s_end)
               
               #print "iteration", idx, ": v_max =", v, ", T4 =", T4
               
               if T4 > epsilon:
                  v_min = v
               elif T4 < epsilon:
                  v_max = v
               else:
                  last_v = v
                  break
               
               # safety
               if idx > 30:
                  break
               
               idx += 1
               
           #T4 = 0


        #print T1, T2, T3, T4, T5, T6, T7
        '''
        print T1, T2, T3, T4, T5, T6, T7

        T1_vals = self.computeTimePeriod(T1, s_0, v_0, a_0, j_max)
        T2_vals = self.computeTimePeriod(T2, T1_vals[3], T1_vals[4], T1_vals[5], 0.0)
        T3_vals = self.computeTimePeriod(T3, T2_vals[3], T2_vals[4], T2_vals[5], -j_max)
        T4_vals = self.computeTimePeriod(T4, T3_vals[3], T3_vals[4], T3_vals[5], 0.0)
        T5_vals = self.computeTimePeriod(T5, T4_vals[3], T4_vals[4], T4_vals[5], -j_max)
        T6_vals = self.computeTimePeriod(T6, T5_vals[3], T5_vals[4], T5_vals[5], 0.0)
        T7_vals = self.computeTimePeriod(T7, T6_vals[3], T6_vals[4], T6_vals[5], j_max)

        print "target pos:", s_end
        print "final pos:", T7_vals[3]
        
        print "target vel:", v_end
        print "final vel:", T7_vals[4]
        
        print "target acc:", a_end
        print "final acc:", T7_vals[5]
        
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.T4 = T4
        self.T5 = T5
        self.T6 = T6
        self.T7 = T7
        '''
        
        return [T1, T2, T3, T4, T5, T6, T7]
        

        ########## work ends here
        
        
    def computeFullTrajectory(self, s_0, v_0, a_0, s_end, v_end, a_end, v_max, a_max, j_max):
    
        self.d = self.computeD(s_0, v_0, a_0, s_end, v_end, a_end)

        epsilon = 0.001
        self.epsilon = epsilon
        
        # If computeD returns 0, there is no error in S, V or A. So all times can be 0.
        if self.d == 0.0:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.jmax = j_max

        result = -1

        # First compute if v_max is exceeded => preperiod needed
        preperiod_needed = False
        T_to_a0 = abs(a_0 / j_max)
        v_peak = self.computeVelocity( T_to_a0, v_0, a_0, -self.d*j_max ) 
        if self.d < 0:
            if v_peak < -v_max:
                preperiod_needed = True
                self.preperiodD = -self.d
                #print "preperiod needed"
        else:
            if v_peak > v_max:
                preperiod_needed = True
                self.preperiodD = self.d
                #print "preperiod needed"

        preperiod_needed = False

        if preperiod_needed:

            # preperiod is needed when the a_0 causes velocity to exceed v_max.
            # it should bring a_0 towards 0 such that v_max is no longer exceeded

            # This computes the preperiod.
            # T1 to T3 computes the periods to reach v_max and a=0.
            # a=0 is maybe not necessary for the preperiod.
            # a might still have a value when ending the preperiod.
            # maybe we can compute back from v_max to the preperiod.
            # if we find the a_end for the preperiod, we can compute it.
            preperiod = self.computeT1T2T3(v_0, a_0, -v_max, a_max, j_max)

            # If T3 < 0, switch the preperiodD
            if preperiod[2] < -self.epsilon:
                self.preperiodD = self.d

            T1_vals = self.computeTimePeriod(preperiod[0], s_0, v_0, a_0, j_max*self.preperiodD)
            T2_vals = self.computeTimePeriod(preperiod[1], T1_vals[3], T1_vals[4], T1_vals[5], 0.0)
            T3_vals = self.computeTimePeriod(preperiod[2], T2_vals[3], T2_vals[4], T2_vals[5], -j_max*self.preperiodD)

            new_s_0 = T3_vals[3]
            new_v_0 = T3_vals[4]
            new_a_0 = T3_vals[5]

            if self.d < 0:
                if new_s_0 < s_end:
                    #print "preperiod exceeds target!"
                    result = list(preperiod)
                    result.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
                    return result
            else:
                if new_s_0 > s_end:
                    #print "preperiod exceeds target!"
                    result = list(preperiod)
                    result.extend([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
                    return result


            newresult = self.computeTrajectory(new_s_0, new_v_0, new_a_0, s_end, v_end, a_end, v_max, a_max, j_max)

            if newresult == -1:
                print "ERROR: Added preperiod. Solution still not found."
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
            elif newresult == -2:
                print "ERROR: Not supported. End conditions exceed limits. (Postperiod is needed.)"
                
                '''
                postperiod = self.computeT5T6T7(new_a_0, v_end, a_end, -v_max, a_max, j_max)
                print postperiod
                
                T1_vals = self.computeTimePeriod(postperiod[0], new_s_0, new_v_0, new_a_0, j_max)
                T2_vals = self.computeTimePeriod(postperiod[1], T1_vals[3], T1_vals[4], T1_vals[5], 0.0)
                T3_vals = self.computeTimePeriod(postperiod[2], T2_vals[3], T2_vals[4], T2_vals[5], -j_max)
                
                print "postperiod reached position:", T3_vals[3]
                print "postperiod reached velocity:", T3_vals[4]
                print "postperiod reached acceleration:", T3_vals[5]
                '''
            
            else:
            
                [T1, T2, T3, T4, T5, T6, T7] = newresult

                T1_vals = self.computeTimePeriod(T1, new_s_0, new_v_0, new_a_0, j_max*self.d)
                T2_vals = self.computeTimePeriod(T2, T1_vals[3], T1_vals[4], T1_vals[5], 0.0)
                T3_vals = self.computeTimePeriod(T3, T2_vals[3], T2_vals[4], T2_vals[5], -j_max*self.d)
                T4_vals = self.computeTimePeriod(T4, T3_vals[3], T3_vals[4], T3_vals[5], 0.0)
                T5_vals = self.computeTimePeriod(T5, T4_vals[3], T4_vals[4], T4_vals[5], -j_max*self.d)
                T6_vals = self.computeTimePeriod(T6, T5_vals[3], T5_vals[4], T5_vals[5], 0.0)
                T7_vals = self.computeTimePeriod(T7, T6_vals[3], T6_vals[4], T6_vals[5], j_max*self.d)
                
                #print "final reached position:", T7_vals[3]
                #print "final reached velocity:", T7_vals[4]
                #print "final reached acceleration:", T7_vals[5]
                
                # New result with preperiod
                result = list(preperiod)
                result.extend(newresult)

        else: #preperiod not needed
            result = self.computeTrajectory(s_0, v_0, a_0, s_end, v_end, a_end, v_max, a_max, j_max)

            if result == -2:
                print "ERROR: Not supported. End conditions exceed limits. (Postperiod is needed.)"
                result = -1

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
