""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# 
# FALCONS // Jan Feitsma, August 2017



# python includes
# none

# package includes
from gamestate import refboxState
import events


# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time


class analyzeRefbox():
    def __init__(self, statistics):
        self.statistics = statistics
        self.currentTimeStamp = None
        self.currentState = None
        self.previousTimeStamp = None
        self.previousState = refboxState() # need intialization to avoid checking for first iteration everytime 
        self.lastSpecial = None
        
    def update(self, state, timeStamp):
        """
        Analyze the refbox state at given timeStamp.
        Called by analyzer on some fixed frequency.
        Will check for state change and update statistics, if applicable.
        """
        # store for use in member functions
        self.currentTimeStamp = timeStamp
        self.currentState = state
        # register active time
        if self.currentState.active:
            self.statistics.match['activeTime'] += (self.currentTimeStamp - self.previousTimeStamp)
        # check for new signal
        (changed, rbSignal) = self.checkStateChange()
        if changed:
            # register / process the signal
            # consider that a 'special' signal (like throwin-prepare) may be given multiple times consecutively
            # due to for instance re-positioning a ball which a robot has bumped into, 
            # so it only counts for statistics when START is given
            events.info("refbox signal: " + rbSignal, level=1, timeStamp=timeStamp)
            if self.isSpecial(rbSignal):
                self.lastSpecial = rbSignal
            if rbSignal == "START":
                self.updateStats(self.lastSpecial)
                self.lastSpecial = None
            # count goal+ and repair-out signals (NOTE: these are sometimes forgotten or misclicked by refbox operators)
            teamIdx = self.determineTeamIdx(rbSignal)
            if "Repair" in rbSignal:
                self.statistics.teams[teamIdx]['repairs'] += 1
                # (over)correct the AV score calculation, assuming the robots keeps generating data
                self.statistics.teams[teamIdx].avScore -= 30.0 * 5 # Hz
                if self.statistics.teams[teamIdx].avScore < 0.0:
                    self.statistics.teams[teamIdx].avScore = 0.0
            if "Goal+" in rbSignal:
                self.statistics.teams[teamIdx]['goals'] += 1
            if "Goal-" in rbSignal:
                self.statistics.teams[teamIdx]['goals'] -= 1
            if "Penalty" in rbSignal:
                self.statistics.teams[teamIdx]['penalties'] += 1
        # update previous
        self.previousTimeStamp = timeStamp
        self.previousState = state

    def checkStateChange(self):
        """
        Check for new refbox signal, i.e. state change.
        Return two outputs: (b, s)
        where b is a boolean indicating state change, and s is the new signal (None otherwise).
        """
        s = self.currentState.lastSignal
        if s != self.previousState.lastSignal:
            trace("refbox state changed from %s to %s", self.previousState.lastSignal, s)
            return (True, s)
        return (False, None)
        
    def isSpecial(self, rbSignal):
        """
        Check if refbox signal is 'special' in the sense that it may add to the statistics of a team,
        in case a START follows.
        """
        if "Throw In" in rbSignal:
            return True
        if "Kickoff" in rbSignal:
            return True
        if "Freekick" in rbSignal:
            return True
        if "Goalkick" in rbSignal:
            return True
        if "Corner" in rbSignal:
            return True
        return False

    def determineTeamIdx(self, rbSignal):
        if "MAGENTA" in rbSignal:
            return 1
        if "CYAN" in rbSignal:
            return 0
        return None # should trigger error
        
    def updateStats(self, rbSignal):
        """
        Update statistics counters.
        """
        trace("rbSignal=%s", rbSignal)
        if rbSignal == None:
            # do nothing - last rbSignal could have been a dropBall or penalty or something we do not understand
            return
        teamIdx = self.determineTeamIdx(rbSignal)
        if "Throw In" in rbSignal:
            self.statistics.teams[teamIdx]['throwins'] += 1
        if "Kickoff" in rbSignal:
            self.statistics.teams[teamIdx]['kickoffs'] += 1
        if "Freekick" in rbSignal:
            self.statistics.teams[teamIdx]['freekicks'] += 1
        if "Goalkick" in rbSignal:
            self.statistics.teams[teamIdx]['goalkicks'] += 1
        if "Corner" in rbSignal:
            self.statistics.teams[teamIdx]['corners'] += 1

