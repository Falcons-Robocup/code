#!/usr/bin/env python   
#
# FALCONS // Jan Feitsma, November 2017


from __future__ import division
from collections import namedtuple, defaultdict
import numpy
import datetime, pytz, time
from copy import copy
from math import pi
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D




class valueStore:
    # store all timed data for a single component
    # make data accessible as namedtuple

    def __init__(self, fn):
        self.data = namedtuple('unusedName', 't ' + fn)
        for it in range(len(self.data._fields)):
            setattr(self.data, self.data._fields[it], [])
        self.empty = copy(self.data)

    def add(self, timeStamp, values):
        values = [timeStamp] + values
        for it in range(len(self.data._fields)):
            getattr(self.data, self.data._fields[it]).append(values[it])

    def prep(self):
        # convert to numpy arrays for efficient masking in get()
        for it in range(0, len(self.data._fields)):
            lst = getattr(self.data, self.data._fields[it])
            if len(lst) and isinstance(lst[0], str):
                setattr(self.data, self.data._fields[it], numpy.array(lst, dtype='string'))
            else:
                setattr(self.data, self.data._fields[it], numpy.array(lst, dtype=numpy.float))
    
    def get(self, timeRange = [0, 1e99]):
        if (len(self.data.t) == 0):
            return self.empty
        mask = ((self.data.t >= timeRange[0]) & (self.data.t <= timeRange[1]))
        result = namedtuple('unusedName', self.data._fields)
        for it in range(len(self.data._fields)):
            setattr(result, self.data._fields[it], getattr(self.data, self.data._fields[it])[mask])
        #print "debug", len(self.data.t), len(result.t), timeRange, self.data.t[0]
        # convert timestamps to matplotlib timestamps? TODO this needs rework
        result.t = matplotlib.dates.date2num([f2dt(t) for t in result.t])
        return result

    def limit(self, timeRange):
        if (len(self.data.t) == 0):
            return
        mask = ((self.data.t >= timeRange[0]) & (self.data.t <= timeRange[1]))
        for it in range(len(self.data._fields)):
            setattr(self, self.data._fields[it], getattr(self.data, self.data._fields[it])[mask])


class eventStore:
    # store all events

    def __init__(self):
        self.elem = namedtuple('event', ['t', 'type', 'details'])
        self.data = []

    def add(self, timeStamp, eventType, details=""):
        self.data.append(self.elem(timeStamp, eventType, details))

    def get(self):
        return self.data
                
    def limit(self, timeRange):
        self.data = [e for e in self.data if (e.t >= timeRange[0]) and (e.t <= timeRange[1])]


def getPhases(timevalues, states):
    # helper to identify 'phases' in a sequence of booleans
    result = []
    ts = None
    inPhase = False
    prevT = timevalues[0]
    for t, state in zip(timevalues, states):
        if state == True:
            if not inPhase:
                # start phase
                ts = 0.5 * (prevT + t)
            inPhase = True
        else:
            if inPhase:
                # end phase
                te = 0.5 * (prevT + t)
                result.append((ts, te - ts))
            inPhase = False
        prevT = t
    # make sure to close last phase
    if inPhase:
        result.append((ts, t - ts))
    return result


# helper to create background colors
def addBackgroundBlocks(ax, timevalues, states, y, height, color, label):
    first = True
    if len(timevalues) == 0:
        return
    for (t, duration) in getPhases(timevalues, states):
        if first:
            ax.add_patch(patches.Rectangle((t, y), duration, height, facecolor=color, edgecolor="none", label=label))
        else:
            ax.add_patch(patches.Rectangle((t, y), duration, height, facecolor=color, edgecolor="none"))
        first = False


# helper to make Falcons timestamp (epoch 2014) readable
EPOCH = 1388534400 # TODO put somewhere common
def timeStr(t):
    s = str(datetime.datetime.fromtimestamp(t + EPOCH))
    return s[11:19] # second resolution
    
def offsetHour():
    z = time.timezone
    if time.daylight:
        z = time.altzone
    return int(z / 3600)
        
def toTimeStamp(dt, epoch=datetime.datetime(1970,1,1,tzinfo=pytz.UTC)):
    # TODO solve timezone magic properly ...
    td = dt - epoch
    # return td.total_seconds()
    return (td.microseconds + (td.seconds + td.days * 86400) * 10**6) / 10**6 

def convertTimeStr(s, t=datetime.datetime.now()):
    # given a string, produce a Falcons float64 timestamp
    try:
        # direct float conversion
        f = float(s)
        return f
    except:
        # parse string, assume HH:MM:SS
        parts = s.split(':')
        t = t.replace(hour=int(parts[0]))
        t = t.replace(minute=int(parts[1]))
        t = t.replace(second=int(parts[2]))
        return toTimeStamp(t) - EPOCH

def f2dt(t):
    # convert falcons timestamp to datetime
    return datetime.datetime.fromtimestamp(t + EPOCH)
    
    
def get_ax_size(ax, fig):
    # https://stackoverflow.com/questions/19306510/determine-matplotlib-axis-size-in-pixels
    bbox = ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    width, height = bbox.width, bbox.height
    width *= fig.dpi
    height *= fig.dpi
    return width, height
    
