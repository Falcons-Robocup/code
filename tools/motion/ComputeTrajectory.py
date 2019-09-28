#!/usr/bin/env python
# Jan Feitsma, October 2017
#
# Implementing the algorithm from the following paper:
# 
#     Planning High Order Trajectories with General Initial and 
#        Final Conditions and Asymmetric Bounds
#     2014, Ezair/Tassa/Shiller
# 
# The algorithm is online, able to handle general initial and final conditions,
# of arbitrary control order (using recursion).
# Complexity: 3rd-order problems are solved within a second, but beyond this, runtime grows (super-)exponentially.
# 
# The result might be sub-optimal in some cases. 
# (For 4th order, see below. For 3rd order: no sub-optimal result found yet.)
# TODO: how bad is it? Can we work around this, patch/extend the algorithm?
# For example the following case will fail to glue ramp-up and ramp-down together:
#    echo -e '4\n0\n1\n3 5 6 10' | python ComputeTrajectory.py -p -s -c 
#



import argparse
from Segments import *
from Solution import *

EPSILON_BSEARCH = 1e-5
EPSILON_V0 = 1e-3



class State(list):
    """
    A State is simply a list of values.
    For an m-th order control problem, the list contains exactly m values.
    """
    def __init__(self, n, *args):
        list.__init__(self, *args)
        self.n = n
        for i in range(len(self)):
            self[i] = float(self[i])
        # initialize other values to zero
        while len(self) < self.n:
            self.append(0.0)
        
    def shift(self):
        return State(self.n-1, self[1:])



class Bounds():
    """
    A Bounds object stores the minimum and maximum values for speed, acceleration etc.
    For an m-th order control problem, both lists contain exactly m values.
    """
    def __init__(self, n):
        self.n = n        
        self.min = [0.0] * n
        self.max = [0.0] * n
        
    def shift(self):
        b = Bounds(self.n-1)
        b.min = self.min[1:]
        b.max = self.max[1:]
        return b
        
    def __repr__(self):
        return '(min=%s, max=%s)' % (str(self.min), str(self.max))



def ComputeTrajectory(m, initialState, finalState, bounds):
    """
    The algorithm from Ezair et al (2014).
    """
    # [step 1] check if problem is simple enough for analytical solution
    #if m == 2: # TODO
    #    return AnalyticalSolution2(initialState, finalState, bounds)
    if m == 1:
        return AnalyticalSolution1(initialState, finalState, bounds)
    # [step 2] calculate distance to be covered
    distance = finalState[0] - initialState[0]
    # [step 3] initialize
    vMin = bounds.min[0]
    vMax = bounds.max[0]
    v = vMax + 1
    vHat = None # last feasible solution
    # [step 4] iterate
    done = False
    while not done:
        # [step 5] backup last v
        vLast = v
        # [step 6] choose next v to be tested (binary search)
        v = 0.5 * (vMax + vMin)
        # HACK: avoid division by zero in step 10 (TODO: fine-tune numerical robustness)
        if v == 0.0:
            v = EPSILON_V0
        # [step 7] recurse 'left' side of the problem (ramp-up)
        solutionLeft = ComputeTrajectory(m-1, initialState.shift(), State(m-1, [v]), bounds.shift())
        # [step 8] recurse 'right' side of the problem (ramp-down)
        solutionRight = ComputeTrajectory(m-1, State(m-1, [v]), finalState.shift(), bounds.shift())
        # [step 9] calculate the distances covered on both sides
        distanceLeft = solutionLeft.x.integrate([0, solutionLeft.T])
        distanceRight = solutionRight.x.integrate([0, solutionRight.T])
        # [step 10] calculate cruising segment
        delta = distance - distanceLeft - distanceRight
        cruise = Segment(0, float(delta) / v, Polynomial(1, v, 0))
        # [step 11] check if feasible
        if cruise.duration() >= 0:
            vHat = v
        # TODO: 'stretch' extension (page 13)
        # TODO: in some cases cruise.duration is approximately 0, but instead both sides should be glued together
        # [steps 12-18] evaluate success and steer binary search direction
        if abs(vMax - vMin) <= EPSILON_BSEARCH:
            # binary search can stop since search space size is within tolerance
            if vHat == None: # no feasible solution found
                raise Exception("algorithm failed")
            v = vHat
            vMin = vHat
            vMax = vHat
        elif delta > 0:
            vMin = v
        elif delta < 0:
            vMax = v
        else:
            vLast = v
        done = (vLast == v)
    # [step 19] calculate total duration
    T = solutionLeft.T + cruise.duration() + solutionRight.T
    # [step 20] join speed segments
    # (time offsetting is handled automatically within operator+)
    v = solutionLeft.x + Segments(cruise) + solutionRight.x
    # [step 21] integrate to find the motion profile
    x = v.integrate() + initialState[0]
    # done
    return Solution(m, T, x)
    
    
# CLI
if __name__ == '__main__':
    # option handling
    argp = argparse.ArgumentParser()
    argp.add_argument('-p', '--plot', action='store_true', help='plot the calculated solution')
    argp.add_argument('-q', '--quiet', action='store_true', help='do not prompt for input')
    argp.add_argument('-c', '--coeffs', action='store_true', help='print resulting segments and coefficients')
    argp.add_argument('-s', '--symmetric', action='store_true', help='use symmetric bounds (do not ask for lower bounds)')
    args = argp.parse_args()
    
    # read the problem specification from stdin
    if not args.quiet:
        print "enter problem order (m)         : ",
    m = int(raw_input())
    if not args.quiet:
        print "enter initial state (%d values) : " % (m),
    initialState = State(m, [float(v) for v in raw_input().split()])
    if not args.quiet:
        print "enter final state (%d values)   : " % (m),
    finalState = State(m, [float(v) for v in raw_input().split()])
    if not args.quiet:
        print "enter upper bounds (%d values)   : " % (m),
    bounds = Bounds(m)
    vals = [float(v) for v in raw_input().split()]
    bounds.max[0:len(vals)] = vals
    if args.symmetric:
        bounds.min = [-v for v in bounds.max]
    else:
        if not args.quiet:
            print "enter lower bounds (%d values)   : " % (m),
        vals = [float(v) for v in raw_input().split()]
        bounds.min[0:len(vals)] = vals
        
    # solve
    solution = ComputeTrajectory(m, initialState, finalState, bounds)
    
    # output two numbers: duration and control setpoint
    if not args.quiet:
        print ""
        print "duration: ",
    print "%.3f" % (solution.T)
    if not args.quiet:
        print "setpoint: ",
    print "%.3f" % (solution.setpoint())
    
    # extra output (coefficients, plot)?
    if args.coeffs:
        print solution
    if args.plot:
        solution.plot()
        
