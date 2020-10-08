""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python3 -u


import sys
import analyze_lib as lib
import eventlog
import rdl_adapters
from collections import OrderedDict



class Model():
    """
    Base class to perform common modeling operations.
    """
    def __init__(self, rdl, agent, verbose=True, resampleFrequency=25.0):
        self.agent = agent
        self.verbose = verbose
        self.resampleFrequency = resampleFrequency
        self.handleRDL(rdl) # load if not done already

    def info(self, msg):
        """
        Optionally display progress messages.
        """
        if self.verbose:
            sys.stdout.write(msg)
            sys.stdout.flush()

    def handleRDL(self, rdl):
        """
        Load RDL file or store frames if loaded already.
        """
        # allow two kind of arguments: rdl file name (string) or rdl frames (list)
        # useful for instance when analyzing multiple robots, to just load RDL once
        if isinstance(rdl, str):
            self.info("loading rdl ...")
            # load RDL
            rdl = lib.RDLFile(rdl)
            rdl.parseRDL()
            self.info(" done\n")
        self.rdl = rdl
        # resample?
        if self.resampleFrequency != None:
            self.info("resampling data to %.1fHz ..." % (self.resampleFrequency))
            self.rdl = lib.RDLResample(self.rdl, self.resampleFrequency)
            self.info(" done\n")
        # find events
        self.events = eventlog.find_events(self.rdl.frames)

    def dumpdf(self, df, filename='/tmp/dataframe.txt'):
        """
        Debugging: dump a dataframe as ASCII table to file.
        """
        self.info("writing dataframe to file: " + filename + " ...")
        tfile = open(filename, 'w')
        tfile.write(df.to_string())
        tfile.close()
        self.info(" done\n")

    def makeDataFrame(self, *args):
        self.columntypes = OrderedDict()
        self.handles = {}
        def addAdapter(adapter):
            for (k,v) in adapter.columntypes.items():
                self.columntypes[k] = v
            for (k,v) in adapter.handles.items():
                self.handles[k] = v
        addAdapter(rdl_adapters.Common())
        for adapter in args:
            addAdapter(adapter)
        self.dataframe = lib.make_dataframe(self.rdl.frames, self.agent, self.columntypes, self.handles, strict=True)

