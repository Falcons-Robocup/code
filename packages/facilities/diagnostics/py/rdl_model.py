# Copyright 2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
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

