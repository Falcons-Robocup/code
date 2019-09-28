#!/usr/bin/env python
#
# Helper classes for Trajectory Planning prototyping.
# 
# Jan Feitsma, October 2017



from Segments import *
import matplotlib.pyplot as plt
import numpy as np



class Solution():
    """
    A Solution object is a container for the solution of the algorithm.
    m: order of the control problem.
    T: the total duration of the computed trajectory.
    x: the segments describing the motion profile x(t).
    From this, the current control setpoint can be calculated by evaluating the m-th order derivative of x at t=0.
    """
    def __init__(self, m, T, x):
        self.m = m
        self.T = T
        self.x = x
        
    def setpoint(self):
        # calculate m-th order derivative of first segment
        p = self.x.data[0].poly
        for i in range(self.m):
            p = p.deriv()
        # evaluate
        return p(0)
    
    def __repr__(self):
        return 'm=%d T=%.5fs\n%s' % (self.m, self.T, str(self.x))
        
    def plot(self):
        [ts, te] = self.x.domain()
        t = np.linspace(ts, te, 400)
        f, axarr = plt.subplots(1+self.m, sharex=True)
        s = self.x
        titles = ['order %d' % (iOrder) for iOrder in range(10)]
        titles[0] = 'position'
        titles[1] = 'velocity'
        titles[2] = 'acceleration'
        titles[3] = 'jerk'
        for iOrder in range(1+self.m):
            x = [s.evaluate(v) for v in t]
            axarr[iOrder].plot(t, x)
            axarr[iOrder].set_title(titles[iOrder])
            # calculate derivative for next plot
            s = s.derivative()
        plt.show()
        
        

def AnalyticalSolution1(initialState, finalState, bounds):
    """
    Analytical solution to the first-order control problem.
    By recursion of ComputeTrajectory, we always end in some analytical solution, 
    which may be this one, or e.g. a 2nd order variant.
    """
    xStart = initialState[0]
    xEnd = finalState[0]
    vMin = bounds.min[0]
    vMax = bounds.max[0]
    # construct Solution
    if xStart < xEnd:
        v = vMax
    elif xStart > xEnd:
        v = vMin
    else:
        raise Exception('xStart==xEnd==%.5f' % (xStart))
    if v == 0:
        raise Exception('cannot solve for v==0')
    T = (float(xEnd) - xStart) / v
    x = Segment(0, T, Polynomial(1, xStart, v))
    return Solution(1, T, Segments(x))
    
# TODO: work out or find somewhere AnalyticalSolution2


