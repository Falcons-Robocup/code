# Copyright 2019 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
import numpy
from collections import defaultdict



class Stats:
    def __init__(self, name, values):
        self.name = name
        # robustness for None values (TODO: NaN?)
        values = [v for v in values if v != None]
        self.values = numpy.array(values)
        self.n = len(self.values)
        self.mean = numpy.mean(self.values)
        self.sigma = numpy.std(self.values)
        self.min = numpy.min(self.values)
        self.max = numpy.max(self.values)
    def __str__(self):
        return "(%s: n=%d, mean=%.2f, std=%.2f)" % (self.name, self.n, self.mean, self.sigma)

class Series:
    def __init__(self, name = None):
        self.name = name
        self.t = []
        self.values = []
    def insert(self, t, value):
        self.t.append(t)
        self.values.append(value)
    def stats(self):
        return Stats(self.name, self.values)
    def getTime(self):
        return self.t;
    def getValues(self):
        return self.values;
    def __str__(self):
        return str(Stats(self.name, self.values))

class MultiSeries:
    def __init__(self):
        self.data = {}
    def insert(self, label, t, value):
        if not self.data.has_key(label):
            self.data[label] = Series(label)
        self.data[label].insert(t, value)
    def get(self, label):
        return self.data[label]
    def __str__(self):
        s = ''
        for label in sorted(self.data.keys()):
            s += str(self.data[label]) + '\n'
        return s

class WorldModelAnalyzer:
    def __init__(self, rdl, agent):
        self.rdl = rdl
        self.agent = agent

    def ballStatsBetweenAge(self, ageMin = None, ageMax = None):
        result = MultiSeries()
        for frame in self.rdl.frames:
            if (ageMin is None or frame.age >= ageMin) and (ageMax is None or frame.age <= ageMax):
                # get list of balls
                balls = frame.data[self.agent]["BALLS"].value
                t = frame.age
                (x, y, z, vx, vy, vz, conf) = (None, None, None, None, None, None, None)
                if len(balls) > 0:
                    ball = balls[0]
                    x = ball[0][0]
                    y = ball[0][1]
                    z = ball[0][2]
                    vx = ball[1][0]
                    vy = ball[1][1]
                    vz = ball[1][2]
                    conf = ball[2]
                    # TODO compress
                result.insert('x', t, x)
                result.insert('y', t, y)
                result.insert('z', t, z)
                result.insert('vx', t, vx)
                result.insert('vy', t, vy)
                result.insert('vz', t, vz)
                result.insert('conf', t, conf)
        return result
        
    def obstacleStatsBetweenAge(self, ageMin = None, ageMax = None):
        result = defaultdict(lambda: MultiSeries())
        for frame in self.rdl.frames:
            if (ageMin is None or frame.age >= ageMin) and (ageMax is None or frame.age <= ageMax):
                for obst in frame.data[self.agent]["OBSTACLES"].value:
                    t = frame.age
                    ((x, y), (vx, vy), conf, id) = obst
                    result[id].insert('x', t, x)
                    result[id].insert('y', t, y)
                    result[id].insert('vx', t, vx)
                    result[id].insert('vy', t, vy)
                    result[id].insert('conf', t, conf)
        return result

    def obstacleStatsBetweenAge(self, ageMin = None, ageMax = None):
        result = defaultdict(lambda: MultiSeries())
        for frame in self.rdl.frames:
            if (ageMin is None or frame.age >= ageMin) and (ageMax is None or frame.age <= ageMax):
                for obst in frame.data[self.agent]["OBSTACLES"].value:
                    t = frame.age
                    ((x, y), (vx, vy), conf, id) = obst
                    result[id].insert('x', t, x)
                    result[id].insert('y', t, y)
                    result[id].insert('vx', t, vx)
                    result[id].insert('vy', t, vy)
                    result[id].insert('conf', t, conf)
        return result

    def framesBetweenAge(self, ageMin, ageMax):
        return [frame for frame in self.rdl.frames if frame.age >= ageMin and frame.age <= ageMax]
        

