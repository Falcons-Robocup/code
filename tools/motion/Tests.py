#!/usr/bin/env python
#
# Testing ComputeTrajectory. 
#
# Jan Feitsma, October 2017


from ComputeTrajectory import *


# test suite
if __name__ == '__main__':
    # a quick self-test
    b = Bounds(1)
    b.max[0] = 2
    S = ComputeTrajectory(1, State(1), State(1, [7]), b)
    assert(S.T == 3.5)
    # examples from Ezair paper, page 10
    # TODO: 4th order doesn't work - no 3rd order cruising phase...
    durations = [0, 0, 0.141, 0.252] #, 0.350]
    tol = 1e-3
    for m in range(2, len(durations)):
        b = Bounds(m)
        for k in range(1, m+1):
            b.min[k-1] = -10**(k+2)
            b.max[k-1] =  10**(k+2)
        S = ComputeTrajectory(m, State(m), State(m, [50]), b)
        print S
        #S.plot()
        assert(abs(S.T - durations[m]) < tol)
    print "all OK"    
