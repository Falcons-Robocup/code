#!/usr/bin/env python
#
# Helper classes for Trajectory Planning prototyping.
# 
# Jan Feitsma, October 2017




from numpy import poly1d 
import copy

EPSILON_DURATION = 1e-7
EPSILON_DT = 1e-6


class Polynomial(poly1d):
    """
    Facade for numpy poly1d.
    Input arguments are treated from low order to high, so offset comes first.
    However numpy turns this around.
    """
    def __init__(self, order, *args):
        coef = [0] * (order+1)
        for i in range(len(args)):
            coef[-1-i] = args[i]
        poly1d.__init__(self, coef)



class Segment():
    """
    A Segment is a polynomial defined on some interval [t1, t2].
    """
    def __init__(self, tStart, tEnd, poly):
        self.tStart = tStart
        self.tEnd = tEnd
        self.poly = poly
        
    def duration(self):
        return self.tEnd - self.tStart



class Segments():
    """
    A Segments object is a sequence of Segment objects, which are chosen such to be continuous  
    up to some order at segment transitions.
    """
    def __init__(self, s=None):
        self.data = []
        if s != None:
            self.data.append(s)
        
    def __add__(self, other):
        result = self
        if isinstance(other, float):
            # offset each segment
            for i in range(len(result.data)):
                result.data[i].poly[0] += other
        elif isinstance(other, Segments):
            if other.duration() > EPSILON_DURATION:
                # join
                T = self.duration()
                for i in range(len(other.data)):
                    o = other.data[i]
                    o.tStart += T
                    o.tEnd += T
                    result.data.append(o)
        else:
            print other
            raise Exception("unsupported use of + on above object")
        # collapse empty segments - TODO: it still can happen that we end up with (almost-)empty segments... how to make this robust?
        if self.duration() < EPSILON_DURATION:
            result = other
        return result
        
    def duration(self):
        return self.data[-1].tEnd - self.data[0].tStart
        
    def domain(self):
        return (self.data[0].tStart, self.data[-1].tEnd)
        
    def evaluate(self, t):
        for i in range(len(self.data)):
            if t < EPSILON_DT + self.data[i].tEnd:
                return self.data[i].poly(t-self.data[i].tStart)
        raise Exception("t=%.6f out of bounds [%.6f, %.6f]" % (t, self.data[0].tStart, self.data[-1].tEnd))
        
    def __repr__(self):
        s = 'segment    tStart      tEnd   poly\n'
        for iSegment in range(len(self.data)):
            segment = self.data[iSegment]
            s += '%7d %9.5f %9.5f' % (iSegment, segment.tStart, segment.tEnd)
            for coef in reversed(segment.poly.coeffs):
                s += ' %9.5f' % (coef)
            s += '\n'
        return s
        
    def integrate(self, domain=None):
        # construct a new Segments object by symbolic integration
        result = copy.deepcopy(self)
        for i in range(len(result.data)):
            result.data[i].poly = result.data[i].poly.integ()
            # set offset coefficient
            if i > 0:
                offset = result.data[i-1].poly(result.data[i-1].tEnd - result.data[i-1].tStart)
                result.data[i].poly.coeffs[-1] = offset
        # if asked to evaluate on some domain numerically, then do so
        if domain != None:
            return result.evaluate(domain[1]) - result.evaluate(domain[0])
        # otherwise return the integrated Segments
        return result
        
    def derivative(self):
        result = copy.deepcopy(self)
        for i in range(len(result.data)):
            result.data[i].poly = result.data[i].poly.deriv()
        return result
        

