#!/usr/bin/env python3
#
# Used in tracing to figure out which file name to use.
# This script is not supposed to be run standalone!

# EKPC 2018-09-13 Creation

import os
import subprocess
import re

### NOTE: If a process is logged as "unknown", please check that the process is started with 'processStart A0 <proc>'


if __name__ == "__main__":
    pid = os.getpid()

    # pstree -pls <PID>
    # ->
    # systemd(1)---systemd(982)---gnome-terminal-(1678)---bash(2115)---processStart(2940)---restartWrapper(2954)---outputwrapper2(2956)---teamplay_main(3017)

    # Get the PID of processStart
    cmd = "pstree -pls %d" % (pid)
    psTreeOutput = str(subprocess.check_output(cmd, shell=True)).strip()

    psOutput = " "

    for process in psTreeOutput.split("---"):
        if "processStart" in process:

            # process == processStart(2940)
            # get PID
            p = re.compile("\(([0-9]+)\)")
            ppid = p.search(process).group(1) # 2940

            # ps --no-headers <PID>
            # ->
            # 2940 pts/1    S+     0:00 /bin/bash /home/robocup/falcons/code/packages/processManager/processStart tp
            cmd = "ps --no-headers %s" % (ppid)
            psOutput = subprocess.check_output(cmd, shell=True).decode("utf-8").strip()

            break


    # Grab last word from psOutput
    #print(psOutput.split(" "))
    componentName = psOutput.split(" ")[-1]
    teamAndRobotID = psOutput.split(" ")[-2]
    
    # sanity checks, it may be that processStart was not used, or ...
    fallBack = False
    if len(teamAndRobotID) > 2:
        fallBack = True
    if len(teamAndRobotID) == 2:
        if teamAndRobotID[0] not in "AB":
            fallBack = True

    # if fallBack needed, print unknown
    if fallBack:
        print("trace_A0_unknown")
    else:
        # legacy behavior
        print("trace_%s_%s" % (teamAndRobotID, componentName))

