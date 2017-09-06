#To check a robot's latency, we introduced two TRACE calls:
#- One in vision when he sees the ball.
#- One in peripheralsInterface when a new target setpoint is received.
#
#Running this tool on the robot will
#1. grep for "LATENCY" in the latest log dir.
#2. sort the TRACE calls by timestamp.
#3. Print the output to stdout. Make sure you pipe it to somewhere.

import subprocess
import re

if __name__ == "__main__":
    
    # Find the latest log dir
    logdir = subprocess.check_output("newest_logdir.py").strip()
    
    # Do the grep on LATENCY
    cmd = "grep --no-filename LATENCY %s/*" % logdir
    output = subprocess.check_output(cmd, shell=True).split('\n')

    # Sort the lines
    output.sort()

    for line in output:
        # Replace "/home/robocup/falcons/code/packages" with "" for better readability
        print line.replace("/home/robocup/falcons/code/packages", "")
