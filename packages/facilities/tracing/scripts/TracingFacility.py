# Copyright 2020 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
import tarfile
import subprocess
from datetime import datetime
import re
import json
import traceback
import os

remoteTraceFilesDir = "/var/tmp/remoteTraceFiles"

class TraceFile:
    
    def __init__(self, filepath):

        self.filepath = filepath
        self.directory = self.getDirectory()
        self.filename = self.getFileName()
        self.process = self.getProcess()
        self.timeAsInt = self.getTime()
        self.sliceNr = self.getSliceNr()

    def getDirectory(self):
        # /var/tmp/falcons_control_20190623_145815/trace_A1_pp.json.20200530_152205.slice1.tar.gz
        # =>
        # ['', 'var', 'tmp', 'falcons_control_20190623_145815']
        directories = self.filepath.split("/")[0:-1]

        # ['', 'var', 'tmp', 'falcons_control_20190623_145815']
        # =>
        # /var/tmp/falcons_control_20190623_145815
        directory = "/".join( directories )

        return directory

    def getFileName(self):
        # /var/tmp/falcons_control_20190623_145815/trace_A1_pp.json.20200530_152205.slice1.tar.gz
        # =>
        # trace_A1_pp.json.20200530_152205.slice1.tar.gz
        filename = self.filepath.split("/")[-1]

        return filename

    def getProcess(self):

        # trace_A1_pp.json.20200530_152205.slice1.tar.gz
        # =>
        # trace_A1_pp
        process = self.filename.split(".")[0]

        return process

    def getTime(self):
        if "slice" in self.filename:
            # trace_A1_pp.json.20200530_152205.slice1.tar.gz
            # =>
            # 20200530_152205
            tsStr = self.filename.split(".")[2]
            
            # 20200530_152205
            # =>
            # 152205
            timeStr = tsStr.split("_")[1]

            return int(timeStr)
        else:
            return 0

    def getSliceNr(self):
        if "slice" in self.filename:
            # trace_A1_pp.json.20200530_152205.slice112.tar.gz
            # =>
            # slice112
            sliceStr = self.filename.split(".")[3]

            # slice112
            # =>
            # 112
            sliceNrStr = sliceStr.replace("slice", "")

            return int(sliceNrStr)
        else:
            return -1

    def distanceToTimestamp(self, ts):
        # ts = "HH:MM:SS"
        # tsAsInt = int("HHMMSS")
        # return abs( self.timeAsInt - tsAsInt )

        # "15:22:05" => "152205"
        tsAsStr = ts[0:2] + ts[3:5] + ts[6:8]
        tsAsInt = int(tsAsStr)

        return abs( self.timeAsInt - tsAsInt )

    def __str__(self):
        return self.filepath

    def __repr__(self):
        return self.filepath

#----------------------

def getTimestampFromLogfile(logfile):

    # Open the logfile and read the first line.
    # from this first line, parse the timestamp.
    # return this timestamp

    content = ""
    if logfile.endswith(".tar.gz"):
        # Read file contents from tarfile
        with tarfile.open(logfile) as tfile:
            try:
                for tfilemember in tfile.getmembers():
                    content = tfile.extractfile( tfilemember ).read()
            except IOError as e:
                print("Error while parsing '%s': %s" % (logfile, str(e)))
                exit(1)

    else:
        # Read file contents from textfile
        with open(logfile, "r") as f:
            content = f.read()

    lines = content.split("\n")

    # {"cat":"/home/robocup/falcons/code/packages/pathPlanning/src/PathPlanning.cpp","pid":31458,"tid":1566128064,"ts":1590235971961638,"ph":"E","name":"WRITE_TRACE","args":{}},
    single_line = lines[1]
    ts_str = int( single_line.split(",")[3].split(":")[1] ) # ts in milliseconds
    ts = datetime.fromtimestamp(float(ts_str) / 1000000.0)
    return ts

#----------------------

def promptLogdir(robot):

    # Determine falcons logging directory to use
    # If robot, get it from the robot

    # Prompt the user which logdir to use

    # The following commands give an error: ls: write error: Broken pipe
    # Because a | can not be used in subprocess.call(shell=True)
    # The alternative, using subprocess.Popen(), does not allow wildcards: *
    # We are able to use a | and a * by using ssh.
    #cmd = 'bash -c "head < <(ls -1dt /var/tmp/falco*)"'
    #cmd = 'ls -1dt /var/tmp/falco* | head'
    cmd = 'ssh localhost "ls -1dt /var/tmp/falco* | head"'

    # If robot, SSH to robot to find logdirs
    if robot:
        cmd = 'ssh robocup@%s "ls -1dt /var/tmp/falco* | head"' % (robot)

    logdirs = subprocess.check_output(cmd, shell=True).decode().split("\n")

    # Remove empty string
    logdirs = logdirs[0:-1]

    print("Choose a logging directory to view tracing:")
    idx = 0
    for logdir in logdirs:
        print("%s: %s" % (idx, logdir))
        idx += 1

    logdirIdx = input("--> ")
    logdir = logdirs[ int(logdirIdx) ]

    return logdir

#----------------------

def getTraceFilesFromLogDir(logDir, robot):

    # Find all tracing files in the logDir

    cmd = "ls -tr %s/trace_*" % logDir

    if robot:
        cmd = 'ssh robocup@%s "ls -tr %s/trace_*"' % (robot, logDir)

    logs = subprocess.check_output(cmd, shell=True).decode().split("\n")

    # Remove empty string
    logs = logs[0:-1]

    result = []
    for log in logs:
        # As preprocessing step, remove any slice files that were compressed but not removed
        # This is possible when the software is killed at the moment the `tar` command was running
        # The `rm` command will then no longer be executed

        # trace_A1_pp.json.20200530_152205.slice1 -> trace_A1_pp.json.20200530_152205.slice
        if log[:-1].endswith(".slice"):
            # Remove this file
            cmd = "rm " + log
            if robot:
                cmd = 'ssh robocup@%s "rm %s"' % (robot, log)
            subprocess.call(cmd, shell=True)

        else:
            try:
                traceFile = TraceFile(log)
                result.append( traceFile )
            except Exception:
                traceback.print_exc()
                pass

    return result

#----------------------

def getProcessesFromTraceFiles(traceFiles):

    processes = set()
    for traceFile in traceFiles:

        processes.add( traceFile.process )

    processes = list(processes)
    processes.sort()

    return processes

#----------------------

def promptProcessesToParse(processes):

    # e.g., processes = [ trace_A1_tp, trace_A2_tp ]
    processesToParse = []

    print("Choose a process to view tracing (or more separated by commas: e.g. '2,3,4'):")
    idx = 0
    for process in processes:
        print("%s: %s" % (idx, process))
        idx += 1

    # prompt
    processIdx = input("--> ")

    if "," in processIdx:
        # Remove spaces
        processIdx = processIdx.replace(" ", "")

        # 1,2 -> trace_A1_tp, trace_A2_tp
        indexes = processIdx.split(",")

        for idx in indexes:
            idx = int(idx)
            processesToParse.append( processes[idx] )

    else:

        # 2 -> trace_A2_tp
        processIdx = int(processIdx)
        processesToParse.append( processes[processIdx] )

    return processesToParse

#----------------------

def promptTimestamp(traceFiles):

    # Prompt the timestamp (or 'all')

    maxTs = 0
    minTs = 999999

    # Determine the minimum and maximum timestamp HH:MM:SS
    for traceFile in traceFiles:

        if traceFile.timeAsInt < minTs:
            minTs = traceFile.timeAsInt

        if traceFile.timeAsInt > maxTs:
            maxTs = traceFile.timeAsInt

    # 152205 => "15:22:05"
    maxTsStr = str(maxTs)[0:2] + ":" + str(maxTs)[2:4] + ":" + str(maxTs)[4:6]
    minTsStr = str(minTs)[0:2] + ":" + str(minTs)[2:4] + ":" + str(minTs)[4:6]

    print("Choose a timestamp between %s and %s or type 'all':" % (minTsStr, maxTsStr))
    
    # prompt
    ts = input("[HH:MM:SS]/all --> ")

    if ts == "all":
        return ts
    else:
        # validate input
        pattern = re.compile("^[0-9]{2}:[0-9]{2}:[0-9]{2}$")
        result = pattern.match(ts)

        if result:
            return ts
        else:
            print("Error: input '%s' does not meet the format [HH:MM:SS] and is unequal to 'all'" % (ts))
            exit()

#----------------------

def filterTraceFiles(traceFiles, processesToParse, ts):

    result = []

    # Given a list of traceFiles, make a subselection on processesToParse and timestamp ts
    # traceFiles= [ /var/tmp/falcons_control_20190623_145815/trace_A1_pp.json.20200530_152205.slice1.tar.gz, ... ]
    # processesToParse = [ trace_A1_tp, trace_A1_wm, trace_A1_vc ]
    # ts = "HH:MM:SS" OR "all"

    # 1. Split all traceFiles to a dictionary:
    # traceFilesByProcess = {'trace_A1_pp': ["/var/tmp/falcons_control_20190623_145815/trace_A1_pp.json.20200530_152205.slice1.tar.gz"] }
    traceFilesByProcess = dict()

    # Fill with empty lists for each of processesToParse
    for process in processesToParse:
        traceFilesByProcess[process] = []

    for traceFile in traceFiles:
        if traceFile.process in processesToParse:
            traceFilesByProcess[traceFile.process].append( traceFile )

    # 2. For each process, find traceFile closest to timestamp ts. Add this tracefile and surrounding slices to the result.
    if ts != "all":
        for process in traceFilesByProcess:

            closestTraceFile = None
            closestDistance = 99999999

            for traceFile in traceFilesByProcess[process]:
                if traceFile.distanceToTimestamp(ts) < closestDistance:
                    closestTraceFile = traceFile
                    closestDistance = traceFile.distanceToTimestamp(ts)

            # Add closestTraceFile and surrounding slices to the result
            for traceFile in traceFilesByProcess[process]:
                if abs( traceFile.sliceNr - closestTraceFile.sliceNr ) < 3:
                    result.append( traceFile )
    else:
        # ts == "all". Add all traceFiles.
        for process in traceFilesByProcess:
            for traceFile in traceFilesByProcess[process]:
                result.append( traceFile )

    return result

#----------------------

def downloadTraceFilesFromRobot(traceFiles, robot):

    # If the logfiles are located remotely on a robot, first download the files to a local folder

    # Delete folder contents if it exists, otherwise create folder
    if os.path.exists(remoteTraceFilesDir):
        cmd = "rm %s/*" % (remoteTraceFilesDir)
        subprocess.call(cmd, shell=True)
    else:
        os.mkdir(remoteTraceFilesDir)

    # Copy all remote trace files to local machine
    traceFilesRemoteStr = ",".join([traceFile.filename for traceFile in traceFiles])
    traceFilesDir = traceFiles[0].directory
    # scp 'r6:/var/tmp/falcons_control_20190623_145815/{trace_A6_vc.json.20200530_151243.slice7.tar.gz,trace_A6_vc.json.20200530_151303.slice8.tar.gz}' /var/tmp/remoteTraceFiles
    cmd = "scp 'robocup@%s:%s/{%s}' %s" % (robot, traceFilesDir, traceFilesRemoteStr, remoteTraceFilesDir)
    print("Copying files from robot '%s' to '%s':" % (robot, remoteTraceFilesDir))
    print(cmd)
    subprocess.call(cmd, shell=True)

    # Update logfiles to the copied files on the local machine
    cmd = "ls -1t %s/*" % (remoteTraceFilesDir)
    logfiles = subprocess.check_output(cmd, shell=True).decode().split("\n")

    # Remove empty string
    logfiles = logfiles[0:-1]

    result = []
    for log in logfiles:
        # As preprocessing step, remove any slice files that were compressed but not removed
        # This is possible when the software is killed at the moment the `tar` command was running
        # The `rm` command will then no longer be executed

        # trace_A1_pp.json.20200530_152205.slice1 -> trace_A1_pp.json.20200530_152205.slice
        if log[:-1].endswith(".slice"):
            # Remove this file
            cmd = "rm " + log
            subprocess.call(cmd, shell=True)

        else:
            try:
                traceFile = TraceFile(log)
                result.append( traceFile )
            except Exception:
                traceback.print_exc()
                pass

    return result

#----------------------

def constructJsonObjectFromTraceFile(traceFile):

    # Read file content (either .tar.gz or simple textfile)
    content = ""
    if traceFile.filename.endswith(".tar.gz"):
        # Read file contents from tarfile
        with tarfile.open(traceFile.filepath) as tfile:
            try:
                for tfilemember in tfile.getmembers():
                    content = tfile.extractfile( tfilemember ).read()
            except IOError as e:
                print("Error while parsing '%s': %s" % (traceFile.filename, str(e)))
                exit(1)

    else:
        # Read file contents from textfile
        with open(traceFile.filepath, "r") as f:
            content = f.read()


    # Having read the file into 'content', attempt parse as JSON
    try:
        try:
            logFileAsJSON = json.loads(content)
        except:
            # fallback:
            # If textfile, it was not closed properly, and the last entry may be corrupt:
            # 20729 {"cat":"cAbstractPathPlanning.cpp","pid":10300,"tid":-268437760,"ts":1561299237134432,"ph":"B","name":"cAbstractPathPlanning#iterateBlock","args":{"msg":""}},
            # 20730 {"cat":"/home/robocup/fal
            # Remove the last line (all chars after the last \n)
            lastidx = content.rfind("\n")
            content = content[0:lastidx-1] # '-1' to also remove the ',' from the previous line
            # Add characters to make it valid JSON
            content += "\n]}\n"
            # Try again
            logFileAsJSON = json.loads(content)
    except ValueError as e:
        print("Error parsing JSON on '%s': %s" % (traceFile, e))
        exit(1)

    return logFileAsJSON


#----------------------

def constructJsonObjectFromTraceFiles(traceFiles):

    result = {"traceEvents": []}

    for traceFile in traceFiles:
        print("Parsing '%s'..." % (traceFile.filename))
        traceObj = constructJsonObjectFromTraceFile( traceFile )
        traceObj = augmentTraceObjWithTimestamps(traceObj)
        result["traceEvents"].extend( traceObj["traceEvents"] )

    # Sort traceObj entries by timestamp to enforce chronological order
    result["traceEvents"].sort(key=lambda x: x["ts"])

    return result

#----------------------

def augmentTraceObjWithTimestamps(traceObj):

    for traceEvent in traceObj["traceEvents"]:
        ts = datetime.fromtimestamp( traceEvent["ts"] / 1E6 )
        traceEvent["args"]["ts"] = ts.strftime("%Y-%m-%d %H:%M:%S.%f")
    return traceObj
