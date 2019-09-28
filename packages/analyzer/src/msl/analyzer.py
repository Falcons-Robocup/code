""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Analyze a MSL logging file (.zip).
# Analyzer class: take a dataStream and inspect it.
#
# FALCONS // Jan Feitsma, August 2017



# python includes
# none

# package includes
import events
from statistics import statistics
from analyzeTeam import analyzeTeam
from analyzeRefbox import analyzeRefbox
from analyzeBallPossession import analyzeBallPossession


# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time


class analyzer:
    """
    Analyze given MSL data stream.
    """

    def __init__(self, dataStream):
        self.dataStream = dataStream
        self.statistics = statistics()
        self.statistics.match['fileName'] = dataStream.fileName
        self.statistics.match['startTime'] = dataStream.timeRange()[0] # this timestamp coincides with t=0 in audienceClient playback
        self.verboseLevel = 0
        self.dt = 0.2
        # setup analyzer helper classes
        self.analyzeTeam = []
        for k in [0, 1]:
            self.analyzeTeam.append(analyzeTeam(dataStream.teamNames[k], self.statistics.teams[k]))
        self.analyzeRefbox = analyzeRefbox(self.statistics)
        self.analyzeBallPossession = analyzeBallPossession(self.statistics)
        
    def setVerboseLevel(self, verboseLevel):
        self.verboseLevel = verboseLevel
        events.SHOW_LEVEL = verboseLevel

    # def runLive(self): # TODO start a timed thread? split class hierarchy?
    
    def runFull(self):
        trace("analyze runFull")
        events.info("analyzing ...", level=0)
        events.INDENT = False
        idx = 0
        state = self.dataStream[idx]
        events.T0 = state.timeStamp # print timestamps relative to first refbox signal
        while state != None:
            t = state.timeStamp
            age = [t - v for v in [state.teams[0].timeStamp, state.teams[1].timeStamp, state.refbox.timeStamp]]
            trace("t=%.3f ageTeam0=%.3f ageTeam1=%.3f ageRefbox=%.3f", t, *age)
            self.update(state, t)
            idx += 1
            state = self.dataStream[idx]
        self.finish(t)

    # internal functions below
    
    def update(self, state, timeStamp):
        trace("t=%.3f active=%d", timeStamp, state.refbox.active)
        # inspect refbox signal/state
        self.analyzeRefbox.update(state.refbox, timeStamp)
        # when analyzing team data, only consider ACTIVE game, i.e. last refbox signal must have been START
        # TODO: for instance KPI 'distance driven' should perhaps also take preparation moves into account ...
        # analyze per team
        for team in [0, 1]:
            self.analyzeTeam[team].update(state.teams[team], timeStamp, state.refbox.active)
        # check for ballPossession change events, count passes etc
        self.analyzeBallPossession.update((state.teams[0], state.teams[1]), timeStamp)

    def finish(self, timeStamp):
        pass
        
