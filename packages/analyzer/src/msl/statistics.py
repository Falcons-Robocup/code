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
from collections import OrderedDict

# package includes
# none



class teamStats(OrderedDict):
    def __init__(self):
        OrderedDict.__init__(self)
        teamKPIs = ["teamName", "dataQualityWarnings", "availability", "repairs", "teleports"]
        teamKPIs += ["goals", "kickoffs", "corners", "goalkicks", "freekicks", "throwins", "penalties"]
        teamKPIs += ["distanceDriven", "maxSpeedFit", "maxSpeedOwn"]
        teamKPIs += ["ballLost", "ballAtCenter", "ballAtOwnHalf", "ballAtOpponentHalf", "ballMaxZ"]
        teamKPIs += ["ballEngaged", "shootAttempts", "selfPasses", "passAttempts", "passes", "passSuccessRate", "ballSteals"]
        for x in teamKPIs:
            self[x] = 0
        self.avScore = 0
        self.avMaxScore = 0

    def setNoData(self):
        # pretty-printing statistics: show n/a instead of zero where applicable
        self['dataQualityWarnings'] = None
        self['teleports'] = None
        for x in ["distanceDriven", "maxSpeedFit", "maxSpeedOwn"]:
            self[x] = None
        for x in ["ballLost", "ballAtCenter", "ballAtOwnHalf", "ballAtOpponentHalf", "ballMaxZ"]:
            self[x] = None
        
    def addAvailability(self, numRobots):
        self.avScore += min(5, numRobots) # clip in case team yields too many robots
        self.avMaxScore += 5
        
    def __getitem__(self, key):
        # override for special string formatting
        # do not print units (e.g. [s], m or %) because that makes postprocessing more complex
        if key == "availability":
            if self.avMaxScore:
                return "%.1f" % (100.0 * self.avScore / self.avMaxScore)
            else:
                return None
        # fallback to regular getitem
        return OrderedDict.__getitem__(self, key)
    
class statistics():
    def __init__(self):
        self.teams = []
        self.teams.append(teamStats())
        self.teams.append(teamStats())
        matchKPIs = ["fileName", "customFileName", "activeTime", "startTime", "ballScrums"]
        self.match = OrderedDict()
        for x in matchKPIs:
            self.match[x] = 0
        self.match['customFileName'] = None
            
    def __repr__(self):
        # initialize and define a helper for nice formatting
        result = ''
        def helper(k, what=0):
            if what == 0:
                v = self.match[k]
            else:
                team = what - 1
                v = self.teams[team][k]
                if v == None:
                    return 'n/a'
            if "distance" in k:
                # round to meters
                return int(round(v))
            if k in ["activeTime", "maxSpeedFit", "maxSpeedOwn", "ballMaxZ", "startTime"]:
                return "%.1f" % (v)
            ballCounters = ["ballLost", "ballAtCenter", "ballAtOwnHalf", "ballAtOpponentHalf"]
            if k in ballCounters:
                # convert counters to percentage
                counts = [self.teams[team][c] for c in ballCounters]
                if "None" in str(counts):
                    return "n/a"
                total = sum(counts)
                if total > 0:
                    return "%.0f" % (100.0 * v / total)
            if k == "passSuccessRate":
                v = "n/a"
                if self.teams[team]["passAttempts"] > 0:
                    return "%.0f" % (100.0 * self.teams[team]["passes"] / self.teams[team]["passAttempts"])
            return v
        # match
        k = self.match.keys()
        v = self.match.values()
        for i in range(len(k)):
            v = helper(k[i], 0)
            if v != None:
                result += "%20s %10s\n" % (k[i], v)
        # team-specific
        k = self.teams[0].keys()
        for i in range(len(k)):
            result += "%20s %10s %10s\n" % (k[i], helper(k[i], 1), helper(k[i], 2))
        return result
            
if __name__ == "__main__":
    s = statistics()
    print s
    
