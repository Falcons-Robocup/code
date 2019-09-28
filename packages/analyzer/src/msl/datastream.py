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
from sortedcontainers import SortedDict
from copy import copy
from math import floor

# package includes
# none


# DEV DEBUG
import roslib
roslib.load_manifest('worldModel')
from FalconsTrace import trace
import time



# NOTE: concerning timestamps: 
# * we like to treat timestamps as floats (as returned by time.time()) because it is intuitive and 
#   achieves plenty of resolution, so on the interfaces we expect float timestamps ('timeStampF')
# * internally however, we use millisecond resolution integers because we do not want to get into 
#   trouble using floating point comparisons in data dictionary ('timeStampMS')


class dataStream:
    """
    A dataStream is a base class to store and provide some packet type with timing.
    In addition, a history of packets is kept in memory (possibly with sub-sampling) 
    which is useful for live analysis, rewinding and freezing the feed temporarily.
    """

    def __init__(self, packetType):
        # required argument(s)
        self.packetType = packetType
        self.reset()
        
    def reset(self):
        # internals
        self._data = SortedDict() # packet buffer, keys are timestamps in milliseconds
        self._tStart = 1e99
        self._tEnd = -1e99
    
    def set(self, packet, timeStampF):
        """
        Feed a packet.
        """
        # convert timestamp
        timeStampMS = self._timeF2MS(timeStampF)
        # store
        self._data[timeStampMS] = packet
        self._tStart = min(self._tStart, timeStampF)
        self._tEnd = max(self._tEnd, timeStampF)

    def get(self, timeStampF):
        """
        Return the packet at (or immediately before) given timestamp.
        """
        t0 = time.time()
        # border case: empty -> return default packet
        if len(self._data) == 0:
            return self.packetType()
        # border case: timestamp before first existing packet -> return default packet
        if timeStampF < self.timeRange()[0]:
            return self.packetType()
        # determine index (binary search)
        idx = self.time2idx(timeStampF)
        result = self._data[self._data.iloc[idx]]
        elapsed = time.time()-t0
        if elapsed > 0.0001:
            trace("SLOW!! get #d=%d t=%.3f elapsed=%.6f", len(self._data), timeStampF, elapsed)
            # TODO: if this happens too often: optimization: use next() / pointer to latest packet
        return copy(result)
        
    def has_key(self, timeStampF):
        """
        Check if timestamp is associated with existing packet.
        """
        return self._data.has_key(self._timeF2MS(timeStampF))
    
    def __getitem__(self, key):
        """
        Act as a regular array. Return None for invalid key index.
        Negative keys can be used.
        """
        try:
            return self._data[self._data.iloc[key]]
        except:
            return None
    
    def time2idx(self, timeStampF):
        """
        Find the index of the packet P with timestamp t such that t <= timeStampF
        """
        if self.empty():
            return None
        if timeStampF < self._tStart:
            return None
        # convert timestamp
        timeStampMS = self._timeF2MS(timeStampF)
        # search buffer using binary search, in case timeStamp is not actually an existing key
        pos = self._data.bisect_right(timeStampMS)
        assert(pos > 0)
        return pos-1
    
    def cleanup(self, timeStampF):
        numRemoved = 0
        for k in self._data.keys():
            if 1e-3 * k < timeStampF:
                del self._data[k]
                numRemoved += 1
        self._recalculateRange()
        trace("numRemoved=%d newSize=%d _tStart=%.3f", numRemoved, len(self._data), self._tStart)

    def __len__(self):
        return len(self._data)
        
    def empty(self):
        return len(self._data) == 0
        
    def timeSize(self):
        """
        Return the time (as float) span of the data stream.
        """
        result = self._tEnd - self._tStart
        assert (result < 24*3600) # sanity check: how can a log be longer than 1 day?
        return result

    def timeRange(self):
        """
        Return the time (as float tuple (tStart, tEnd)) range of the data stream.
        """
        return (self._tStart, self._tEnd)
        
    def __repr__(self):
        """
        Show some useful statistics
        """
        if self.empty():
            return "(empty)"
        return "(tStart=%.3f size=%.3f #elem=%d freq=%.1fHz)" % (self.timeRange()[0], self.timeSize(), len(self._data), len(self._data) / max(1, self.timeSize()))
    
    # internal functions below
    
    def _timeF2MS(self, timestamp):
        # using floats is not a good idea for keys in a dict
        # so we use integers with millisecond resolution
        return int(floor(1000 * timestamp))
        
    def _recalculateRange(self):
        if len(self._data):
            self._tStart = min([1e-3 * k for k in self._data.keys()])
            self._tEnd = max([1e-3 * k for k in self._data.keys()])
        else:
            self._tStart = 1e99
            self._tEnd = -1e99
        
