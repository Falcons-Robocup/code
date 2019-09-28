#!/usr/bin/env python
#
# Used in tracing to figure out which file name to use.
# This script is not supposed to be run standalone!
#
# Process tree typically looks like this (ps axf):
#  3219 pts/26   S      0:00          |   |   \_ /bin/bash /home/robocup/falcons/code/packages/processManager/processStart A0 diagRecvNew
#  3236 pts/26   S      0:00          |   |       \_ /bin/bash /home/robocup/falcons/code/packages/coachCommands/outputwrapper2 /var/tmp/falcons_control_20160225_193846 rosrun diagnostics diagCoachListener
#  3241 pts/26   Sl     0:00          |   |           \_ /home/robocup/falcons/code/packages/facilities/diagnostics/bin/diagCoachListener
#  3262 pts/26   S      0:00          |   |               \_ sh -c getProcessId
#  3263 pts/26   S      0:00          |   |                   \_ /bin/bash /home/robocup/falcons/code/scripts/getProcessId
#
# so we need to traverse a few parents up, then get the last word, in this case 'diagRecvNew'.
# 

# EKPC 2018-09-13 Creation

import os
import subprocess

if __name__ == "__main__":
    pid = os.getpid()

    fallBack = False
    psOutput = ""
    i = 0
    while "processStart" not in psOutput:

        if int(pid) == 0:
            print "unknown_0"
            exit()
       
        # Grab parent process ID of this PID until we have the processStart
        cmd = "ps -h -p%s -o ppid,cmd" % (pid) 
        try:
            psOutput = subprocess.check_output(cmd, shell=True).strip()
        except:
            print "unknown_%s" % (pid)
            exit()
        #print psOutput

        pid = psOutput.split(" ")[0]
        #print "new pid:", pid

        i += 1
        if i > 10:
            fallBack = True
            break

    # Grab last word from psOutput
    #print psOutput.split(" ")
    componentName = psOutput.split(" ")[-1]
    teamAndRobotID = psOutput.split(" ")[-2]
    
    # sanity checks, it may be that processStart was not used, or ...
    if len(teamAndRobotID) > 2:
        fallBack = True
    if len(teamAndRobotID) == 2:
        if teamAndRobotID[0] not in "AB":
            fallBack = True
    if fallBack:
        print "trace_A0_unknown"
    else:
        # legacy behavior
        print "trace_%s_%s" % (teamAndRobotID, componentName)

#function getPpid
#{
#    stat=($(</proc/$1/stat))
#    ppid=${stat[3]}  
#    echo $ppid
#}
#
#pid=$$
#maxdepth=6
#for i in `seq 1 $maxdepth` ; do
#    pid=$(getPpid $pid)
#    pattern=`printf "^robocup %6d " $pid`
#    output=`ps -ef | grep "processStart " | grep -v "grep processStart" | grep "$pattern" | awk '{print $NF}'`
#    if [ ! -z "$output" ]; then
#        echo $output
#        exit 0
#    fi
#done
#
#echo "unknown$$"
#
