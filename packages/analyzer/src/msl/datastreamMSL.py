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
import json
from zipfile import ZipFile

# package includes
from gamestate import gameState, worldState, refboxState, robotState
from datastream import dataStream
import events


# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time




class dataStreamMSL(dataStream):
    """
    Store the state of MSL match with timestamps.
    Used by analyzer to inspect the state on some fixed frequency.
    Sub-sampling is applied (typically 5Hz), which is required for getting 
    consistent results and also benefits speed of analysis. 
    Collapse/fuse three data streams (worldState of both teams and refbox) into a single one, 
    which is useful for visualization and analysis - at every timestamp there is one full packet (parts of it may have aged more than others).
    """

    def __init__(self):
        dataStream.__init__(self, gameState)
        self.teamNames = ["unknown", "unknown"]
        self.fileName = "unknown"
        self.verboseLevel = 1

    def setVerboseLevel(self, verboseLevel):
        self.verboseLevel = verboseLevel
        events.SHOW_LEVEL = verboseLevel

    def feedWorld(self, worldState, team=0):
        """
        Feed the world state according to a team (0 or 1).
        Note: worldState contains a timeStamp.
        """
        # collapse by doing get + partialUpdate + set
        packet = self.get(worldState.timeStamp)
        packet.teams[team] = worldState
        self.set(packet, worldState.timeStamp)

    def feedRefbox(self, refboxState):
        """
        Feed the refbox state.
        Note: refboxState contains a timeStamp.
        """
        # collapse by doing get + partialUpdate + set
        packet = self.get(refboxState.timeStamp)
        packet.refbox = refboxState
        trace('feedRefbox active=%d', packet.refbox.active)
        self.set(packet, refboxState.timeStamp)
        
    def load(self, mslzipfile):
        """
        Load MSL .zip file and and store one collapsed data stream.
        Based on loadZipFile from MSL workshop 2016 in Kassel, to which several Carpe Noctem and Falcons 
        team members contributed.
        (forked RefBox2015/AudienceClient/AudienceClient/playback/matchlog.py)
        """
        events.info("loading {0} ...".format(mslzipfile), level=0)
        trace("loading {0} ...".format(mslzipfile))
        self.fileName = mslzipfile
        # load each .msl file as datastream
        teamInt2Char = "AB"
        self.streams = {}
        # TODO support nested zips? messy, rather fix data files..
        with ZipFile(mslzipfile, 'r') as mslzip:
            # find files and parse json
            for f in mslzip.filelist:
                trace("loading {0} ...".format(f.filename))
                events.info("loading {0} ...".format(f.filename), level=1)
                loadOk = False
                for teamIdx in [0, 1]: # try team stream
                    if f.filename.endswith(".{0}.msl".format(teamInt2Char[teamIdx])):
                        jsonData = json.loads(mslzip.read(f.filename).encode("utf-8"))
                        if len(jsonData):
                            self.streams[teamIdx] = self._loadJsonWorld(jsonData, teamIdx)
                        else:
                            events.info("JSON stream is empty", level=2)
                        loadOk = True
                if not loadOk: # must be refbox stream
                    if f.filename.endswith(".msl") and not ".A." in f.filename and not ".B." in f.filename:
                        self.streams['refbox'] = self._loadRefbox(mslzip.read(f.filename).splitlines())
                        loadOk = True
                if not loadOk: 
                    raise Exception('cannot handle file %s' % (f.filename))

    def fuse(self, sampleFrequency):
        """
        Fuse the three data streams and sub-sample at given frequency.
        """
        dt = 1.0 / sampleFrequency
        # collapse three streams into single stream, sub-sample
        trace("collapsing and sub-sampling streams")
        events.info("collapsing and sub-sampling streams at %.1fHz..." % (sampleFrequency), level=0)
        (tStart, tEnd) = self.streams['refbox'].timeRange()
        # TODO reduce tEnd a bit, typically refbox client stays on quite long after a match
        t = tStart
        while t < tEnd:
            trace('iteration start t=%f', t)
            m = gameState()
            for team in [0, 1]:
                if self.streams.has_key(team) and not self.streams[team].empty():
                    m.teams[team] = self.streams[team].get(t)
                    E = t - m.teams[team].timeStamp
                    trace('team=%d, E=%.3f', team, E)
                    assert(E >= -0.002) # millisecond rounding in dataStream ...
            m.refbox = self.streams['refbox'].get(t)
            E = t - m.refbox.timeStamp
            trace('E=%.3f  iteration set', E)
            m.timeStamp = t
            self.set(m, t)
            t += dt
        # final pass to make sure each refbox event is exactly stored, to prevent signals getting lost
        trace("refbox final pass")
        idx = 0
        r = self.streams['refbox'][idx]
        while r != None:
            t = r.timeStamp
            trace("t=%.3f", t)
            m = self.get(t)
            m.timeStamp = t
            m.refbox = r
            self.set(m, t)
            idx += 1
            r = self.streams['refbox'][idx]
        events.info("datastream result: %s" % (self), level=1)
        

    # internal functions below

    def _loadRefbox(self, lines):
        """
        Parse the lines in a refbox log file.
        Contrary to the worldState logging, it is not JSON, but a custom comma-separated format 
        with some intermediate connect/disconnect info in between.
        """
        # example:
        # 1467381507843,11:14(05:25),2nd Half - STOP,c,MAGENTA Corner
        # 1467381513114,11:19(05:25),2nd Half,s,START
        # 1467381559118,12:05(06:11),2nd Half - STOP,S,STOP
        # 1467381564710,12:11(06:11),2nd Half - STOP,T,CYAN Throw In
        # columns are comma separated
        events.info("parsing refbox stream ...", level=2)
        trace("parsing refbox stream")
        result = dataStream(refboxState)
        for line in lines:
            columns = line.split(',')
            if len(columns) == 5:
                # good line, convert columns
                trace('%s', line)
                r = refboxState()
                r.timeStamp = 1e-3 * float(columns[0]) # !! timestamp is already stored in milliseconds..
                r.phase = columns[2].replace(' - STOP', '')
                r.lastSignal = columns[-1] # only store human readable string, ignore coded char
                r.active = (r.lastSignal == "START") # TODO we could consider preparation time quasi-active?
                trace('t=%.3f signal=%s active=%d', r.timeStamp, r.lastSignal, r.active)
                # store - make sure each signal is stored (note that a REPAIR is sent right before START)
                # by messing a little bit with time stamps ...
                t = r.timeStamp
                while result.has_key(t):
                    t += 0.002
                r.timeStamp = t
                result.set(r, t)
        trace("finished parsing refbox stream")
        trace("datastream result: %s", result)
        events.info("datastream result: %s" % (result), level=2)
        return result
        
    def _loadJsonWorld(self, jsonData, teamIdx):
        if len(jsonData) == 0:
            return
        # determine team name (one time should suffice..)
        entry = jsonData[0]
        self.teamNames[teamIdx] = entry['teamName']
        # work through the data
        events.info("parsing JSON for team '{0}' ...".format(self.teamNames[teamIdx]), level=2)
        trace("parsing JSON stream")
        result = dataStream(worldState)
        for entry in jsonData:
            # convert timestamp to float as seconds
            timeStamp = 1e-3 * long(entry['timestamp'])
            # convert worldState JSON to struct
            w = worldState()
            w.timeStamp = timeStamp # TODO warn for out-of-order entries? see for example Water data
            if len(entry['worldstate']['robots']):
                for r in entry['worldstate']['robots']:
                    robotIdx = r['id']
                    pose = r['pose']
                    vel = r['velocity']
                    if "None" in str(vel):
                        # initialize to zeros
                        vel = (0, 0, 0)
                    if not "None" in str(pose):
                        w.robots[robotIdx] = robotState(pose, vel)
                        if r['ballEngaged'] > 0:
                            w.robots[robotIdx].ballEngaged = True
                    # TODO: intention, targetPose
            if len(entry['worldstate']['balls']):
                # select first ball in list
                # TODO warn in case of multiple balls?
                # TODO use confidence instead of assuming they are sorted descendingly?
                ball = entry['worldstate']['balls'][0]['position']
                if not "None" in str(ball):
                    w.ball = entry['worldstate']['balls'][0]['position']
                # ignore ball speed for now
            # feed
            w.valid = True
            result.set(w, timeStamp)
        trace("finished parsing JSON stream")
        trace("loaded %d entries", len(jsonData))
        trace("datastream result: %s", result)
        events.info("datastream result: %s" % (result), level=2)
        return result

