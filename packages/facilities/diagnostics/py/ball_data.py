""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python

import math
import numpy as np
import pandas as pd


def convertTimeStamp(rtdbTimestamp):
    return float(rtdbTimestamp[0]) + 1e-6 * float(rtdbTimestamp[1])


class BallMeasurement():
    def __init__(self, sph, used=None, trackerId=None):
        self.identifier = sph[0]
        self.robotId = self.identifier[0]
        self.timestamp = convertTimeStamp(sph[1])
        self.used = used
        self.trackerId = trackerId
        #self.camSource = sph[2] # obsolete?
        self.confidence = sph[3]
        self.azimuth = sph[4] + math.pi/2.0
        self.elevation = sph[5]
        self.radius = sph[6] # distance
        self.cameraX = sph[7]
        self.cameraY = sph[8]
        self.cameraZ = sph[9]
        self.cameraRz = sph[10] - math.pi/2.0
        #self.color = sph[11] # obsolete?
        # calculate FCS coordinates for plotting in field context
        cart = np.zeros((4,4))
        cart[0][3] = self.radius*math.cos(self.elevation)*math.cos(self.azimuth)
        cart[1][3] = self.radius*math.cos(self.elevation)*math.sin(self.azimuth)
        cart[2][3] = self.radius*math.sin(self.elevation)
        cart[3][3] = 1.0
        translate = np.eye(4)
        translate[0][3] = self.cameraX
        translate[1][3] = self.cameraY
        translate[2][3] = 0
        rotatez = np.eye(4)
        rotatez[0][0] = math.cos(self.cameraRz)
        rotatez[0][1] = -math.sin(self.cameraRz)
        rotatez[1][0] = math.sin(self.cameraRz)
        rotatez[1][1] = math.cos(self.cameraRz)
        M = np.matmul(translate, rotatez)
        resultM = np.matmul(M, cart)
        self.ballFcsX = resultM[0][3]
        self.ballFcsY = resultM[1][3]
        self.ballFcsZ = resultM[2][3] + self.cameraZ
    def id(self):
        # return a unique string for this object
        # note that identifier counting restarts at 10000 in multiCam/observerRtdb
        # if there are many false positives, 10000 is reached within minutes
        return str(self.identifier) + str(self.timestamp)
    def to_dict(self):
        return vars(self)


class BallMeasurementCluster():
    def __init__(self, bmc):
        self.timestamp = convertTimeStamp(bmc[0])
        self.x = bmc[1]
        self.y = bmc[2]
        self.z = bmc[3]
        self.used = bmc[4]
    def id(self):
        return str(self.timestamp)


class BallResult():
    def __init__(self, d, timestamp, agent):
        self.timestamp = convertTimeStamp(timestamp)
        self.trackerId = None # not present in BALLS, only present in diagnostics details
        self.robotId = agent
        self.x = d[0][0]
        self.y = d[0][1]
        self.z = d[0][2]
        self.vx = d[1][0]
        self.vy = d[1][1]
        self.vz = d[1][2]
        self.confidence = d[2]
        # ignore ball possession (d[3], 'owner')
    def id(self):
        # return a unique string for this object
        return str(self.robotId) + str(self.timestamp) + str(self.trackerId)
    def to_dict(self):
        return vars(self)


class BallDiagnostics():
    def __init__(self, balls, timestamp, agent):
        self.timestamp = convertTimeStamp(timestamp)
        self.timestampStr = str(timestamp)
        self.robotId = agent
        # distribute results, measurements and measurements clusters
        self.ballResults = {}
        self.ballMeasurements = {}
        self.ballMeasurementsClusters = {}
        for b in balls:
            # ball result
            br = BallResult(b['result'], timestamp, agent)
            br.trackerId = b['id']
            for v in ['age', 'freshness', 'ownGoodDataRate', 'outliersFraction', 'obf']:
                setattr(br, v, b[v])
            br.mCount = len(b['measurements'])
            br.mcCount = len(b['measurementClusters'])
            self.ballResults[br.trackerId] = br
            # vision measurements associated to this tracker, possibly tagged as outlier
            for bm in b['measurements']:
                bmv = BallMeasurement(bm['m'], bm['used'], br.trackerId)
                # timestamp shift for plotting in timeline (back from 0)
                bmv.t = bmv.timestamp - self.timestamp
                self.ballMeasurements[bmv.id()] = bmv
            # clustered vision measurements
            for bm in b['measurementClusters']:
                bmc = BallMeasurementCluster(bm)
                # timestamp shift for plotting in timeline (back from 0)
                bmc.t = bmc.timestamp - self.timestamp
                self.ballMeasurementsClusters[bmc.id()] = bmc
    def id(self):
        # return a unique string for this object
        return str(self.robotId) + str(self.timestamp)
    def to_dict(self):
        return vars(self)
        

# a BallData object
# * can interpret RTDB items (doesn't matter if they come from live database or RDL frame)
# * has a memory which by default keeps growing (call cleanup to control it)
# * maps raw rtdb data to objects which
#   * do not expose database design choices,
#   * are easy to use in (plotting) client
class BallData():
    def __init__(self):
        self.ballMeasurements = {}
        self.ballResults = {}
        self.ballDiagnostics = {}

    def getBallMeasurements(self, agent = None, useDiag = True):
        # make dataframe
        if useDiag:
            # new default: via diagnostics data (which is large but accurate and fully associated)
            result = pd.DataFrame.from_records([b.to_dict() for d in self.ballDiagnostics.values() for b in d.ballMeasurements.values()])
        else:
            # old data source: raw measurements (a few robot per tick)
            result = pd.DataFrame.from_records([b.to_dict() for b in self.ballMeasurements.values()])
        if agent != None: # optional filter on agent
            result = result[result['robotId'] == agent]
        return result

    def getBallMeasurementClusters(self, agent = None):
        # make dataframe
        result = pd.DataFrame.from_records([d.to_dict() for b in self.ballDiagnostics.ballMeasurementClusters.values() for d in b.values()])
        print(result.to_string())
        raise
        if agent != None: # optional filter on agent
            result = result[result['robotId'] == agent]
        return result

    def getBallResults(self, agent = None, useDiag = True):
        # make dataframe
        if useDiag:
            # new default: via diagnostics data (which is large but accurate and fully associated)
            result = pd.DataFrame.from_records([b.to_dict() for d in self.ballDiagnostics.values() for b in d.ballResults.values()])
        else:
            # old data source: external ball results (only best ball, not any secondary / false positive tracker)
            result = pd.DataFrame.from_records([b.to_dict() for b in self.ballResults.values()])
        print(result.to_string())
        if agent != None: # optional filter on agent
            result = result[result['robotId'] == agent]
        return result

    def feedBallCandidates(self, item):
        for v in item.value:
            bm = BallMeasurement(v)
            bm.timestamp = convertTimeStamp(item.timestamp) # use rtdb timestamp instead of latency-corrected internal timestamp
            self.ballMeasurements[bm.id()] = bm

    def feedBallResults(self, item):
        for v in item.value:
            br = BallResult(v, item.timestamp, item.agent)
            self.ballResults[br.id()] = br

    def feedBallDiagnostics(self, item):
        if len(item.value['balls']):
            b = BallDiagnostics(item.value['balls'], item.value['timestamp'], item.agent)
            self.ballDiagnostics[b.id()] = b

    def feedMatchState(self, item):
        self.timestamp = convertTimeStamp(item.value['currentTime'])

