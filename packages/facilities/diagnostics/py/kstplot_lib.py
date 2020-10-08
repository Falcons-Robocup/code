""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 # common utilities, for reuse in multiple clients

import os, sys
import shutil
import traceback
import falconspy
from rdlLib import RDLFile
import analyze_lib
import rtdb2tools



class FolderWriter():
    def __init__(self, folder="/var/tmp/kstplot"):
        # create work directory
        self.folder = folder
        if os.path.isdir(self.folder):
            shutil.rmtree(self.folder)
        os.mkdir(self.folder)
        # container for file handles
        self.filehandles = {}

    def output(self, line, key):
        """
        Write line to file associated to key.
        """
        # open file if not done already
        if not self.filehandles.has_key(key):
            filename = self.folder + "/" + key + ".txt"
            self.filehandles[key] = open(filename, 'w')
        # write
        self.filehandles[key].write(line + "\n")
        self.filehandles[key].flush()


class LiveDiagnosticsMonitor(FolderWriter, rtdb2tools.RTDBMonitor):
    def __init__(self, agent, folder="/var/tmp/kstplot", frequency=30.0):
        # setup RTDBMonitor
        rtdb2tools.RTDBMonitor.__init__(self, agent=agent, frequency=frequency, rtdbpath=rtdb2tools.RTDB2_DEFAULT_PATH)
        self.prependTimestamp = True # TODO: in playback mode, this is not the original timestamp ... instead one better uses RDLDiagnostics
        self.showOnce = True # continuous sampling is not desired when pausing playback
        # setup FolderWriter, note that it will overrule standard 'print' by RTDBMonitor.output()
        FolderWriter.__init__(self, folder)


class RDLDiagnostics(FolderWriter):
    def __init__(self, rdlfile, agent, folder="/var/tmp/kstplot"):
        # setup FolderWriter
        FolderWriter.__init__(self, folder)
        # setup the rest
        self.agent = agent
        # guess agent from rdl file
        if agent == 0:
            self.agent = analyze_lib.get_agent_from_rdl_filename(rdlfile)
        self.keys = set()
        self.handles = {}
        self.prependTimestamp = True
        # load RDL
        self.rdl = RDLFile(rdlfile)
        self.rdl.parseRDL() # TODO improve slowness and mem consumption

    def subscribe(self, key, handle=str):
        """
        Subscribe to a key. The value may be nicely formatted through a custom function.
        """
        self.keys.add(key)
        self.handles[key] = handle

    def handle(self, key, item):
        """
        Handle a RTDB frame item.
        """
        try:
            valueAsString = self.handles[key](item.value)
        except:
            print("WARNING: blacklisting key " + key + " after callback failure:")
            traceback.print_exc()
            self.keys.remove(key)
            return
        line = valueAsString
        if self.prependTimestamp:
            line = "%d.%06d %s" % (item.timestamp[0], item.timestamp[1], line)
        self.output(line, key)
        
    def run(self):
        nodata = True
        for frame in self.rdl.frames:
            if frame.data.has_key(self.agent):
                agentData = frame.data[self.agent]
                for key in list(self.keys):
                    if agentData.has_key(key):
                        item = agentData[key] # type: RtDBFrameItem
                        self.handle(key, item)
                        nodata = False
        if nodata:
            raise Exception("no matching data found for agent " + str(self.agent))


# generic utility, originally created in tracePlotUtils.py
def getPhases(timevalues, states):
    # helper to identify 'phases' in a sequence of booleans
    # a 'phase' is a pair (timestamp, duration)
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


