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
# Command-line entry point.
#
# FALCONS // Jan Feitsma, August 2017



# python includes
import os
import argparse

# package includes
from datastreamMSL import dataStreamMSL
from analyzer import analyzer
import settings
import events



# helper function
def calculateTimeOffset(dataStream1, dataStream2):
    """
    Determine the time offset between two dataStreamMSL objects
    by looking at specific refbox signals.
    This is needed when fusing hybrid refbox data (clock from refbox CPU) and team data (clock from baseStation CPU).
    """
    # we could just look at 1st half, but for some extra sanity checking & averaging, we use more sync signals
    signals = ['1st half', 'Halftime', '2nd half', 'End Game']
    # helper function
    def findSignal(refboxDataStream, signal):
        idx = 0
        e = refboxDataStream[idx]
        while e != None:
            if e.lastSignal == signal:
                return e.timeStamp
            idx += 1
            e = refboxDataStream[idx]
        return None
    # calculate
    result = 0
    for s in signals:
        t1 = findSignal(dataStream1.streams['refbox'], s)
        t2 = findSignal(dataStream2.streams['refbox'], s)
        result += t2 - t1
    result /= len(signals) # average
    return result

def applyTimeOffset(dstream, timeOffset):
    """
    Correct the time stamps with some offset.
    Also correct timestamp in packets.
    """
    # straightforward yet expensive: modify all timestamps
    # (alternative: store _offset and apply it in the right places)
    oldData = dstream._data
    dstream.reset()
    for (k, v) in oldData.iteritems():
        t = 1e-3 * k + timeOffset
        v.timeStamp += timeOffset
        dstream.set(v, t)
    

# extract statistics from single file
def main(mslfile, verboseLevel=0, sampleFrequency=5.0, customfile=None):
    """
    Load a MSL logging file
    """
    # load the MSL file as data stream
    d = dataStreamMSL()
    d.setVerboseLevel(verboseLevel)
    d.load(mslfile)
    # if given, load custom team data
    # assume it is a Falcons .bag file; teams can implement their own adapters similarly
    if customfile != None:
        if not os.path.isfile(customfile):
            raise Exception("file not found: {}".format(customfile))
        from datastreamBag import dataStreamBag
        d2 = dataStreamBag()
        d2.setVerboseLevel(verboseLevel)
        d2.load(customfile)
        # calculate time offset between refbox CPU and Falcons coach laptop based on refbox streams (1st half signal)
        timeOffset = calculateTimeOffset(d, d2)
        # correct custom data stream with the time offset
        events.info("correcting .bag datastream with %.3fs offset..." % (timeOffset), level=1)
        applyTimeOffset(d2.streams[0], -timeOffset)
        # insert the Falcons data stream
        streamInserted = False
        for teamIdx in [0, 1]:
            if not d.streams.has_key(teamIdx):
                d.streams[teamIdx] = d2.streams[0]
                d.teamNames[teamIdx] = d2.teamNames[0]
                streamInserted = True
        # override existing MSL data stream?
        doOverride = True
        if not streamInserted and doOverride:
            for teamIdx in [0, 1]:
                if d.teamNames[teamIdx] == "ASML Fal":
                    d.streams[teamIdx] = d2.streams[0]
                    d.teamNames[teamIdx] = d2.teamNames[0]
                    streamInserted = True
        assert(streamInserted)
    # fuse the streams at 5Hz
    d.fuse(sampleFrequency)
    # TODO: restrict the time window (options like tStart tEnd half1 half2) here
    # analyze it
    a = analyzer(d)
    if customfile != None:
        a.statistics.match['customFileName'] = d2.fileName
    a.setVerboseLevel(verboseLevel)
    a.runFull()
    return a.statistics



# command line interface
if __name__ == '__main__':

    # argument parsing
    parser     = argparse.ArgumentParser(description='analyze a MSL logging file')
    parser.add_argument('-f', '--frequency', help='sample frequency', type=float, default=settings.DEFAULT_SAMPLE_RATE)
    parser.add_argument('-c', '--custom', help='own team data log file (e.g. Falcons .bag)')
    parser.add_argument('-v', '--verbose', help='verbose', action='store_true')
    # TODO options tStart and tEnd, for cutting out a specific timewindow
    # TODO building on above: options half1 and half2 for cutting out those halves, using timestamps from refbox data stream
    parser.add_argument('-V', '--verboselevel', help='verbose level', type=int, default=0)
    parser.add_argument('logfile', help='MSL .zip log file', default=None)
    args       = parser.parse_args()
    
    # analyze
    if not os.path.isfile(args.logfile):
        raise Exception("file not found: {}".format(args.logfile))
    if args.verbose:
        args.verboselevel = 1
    stats = main(args.logfile, args.verboselevel, args.frequency, args.custom)
    
    # display results
    print stats

