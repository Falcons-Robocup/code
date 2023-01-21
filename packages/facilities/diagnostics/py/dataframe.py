# Copyright 2022 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python3


# python/system libraries
import sys
import datetime
from collections import OrderedDict

# Falcons libraries
import analyze_lib as lib
import rdl_adapters



class DataFrame():
    """
    Convert data from RDL to a pandas dataframe so it can be plotted, modeled or otherwise processed.
    """
    def __init__(self, rdl, agent, verbose=True, resampleFrequency=25.0):
        self.agent = agent
        self.verbose = verbose
        self.resampleFrequency = resampleFrequency
        # load if not done already
        self.handleRDL(rdl)
        # convert to dataframe
        self.makeDataFrame(*rdl_adapters.all())

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
        # TODO? find events
        #self.events = eventlog.find_events(self.rdl.frames)

    def writeAsTXT(self, filename='/tmp/dataframe_<timestamp>.txt'):
        """
        Dump a dataframe as ASCII table to file.
        """
        filename = filename.replace('<timestamp>', datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.info("writing dataframe to file: " + filename + " ...")
        tfile = open(filename, 'w')
        tfile.write(self.dataframe.to_string())
        tfile.close()
        self.info(" done\n")

    def writeAsCSV(self, filename='/tmp/dataframe_<timestamp>.csv'):
        """
        Dump a dataframe as CSV to file.
        """
        filename = filename.replace('<timestamp>', datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        self.info("writing dataframe to file: " + filename + " ...")
        tfile = open(filename, 'w')
        tfile.write(self.dataframe.to_csv())
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

