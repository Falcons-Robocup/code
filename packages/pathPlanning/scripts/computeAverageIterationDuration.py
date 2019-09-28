""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import sys, subprocess, os
from datetime import datetime

if __name__ == "__main__":

    # Find latest logdir
    cmd = "python /home/robocup/falcons/code/packages/coachCommands/newest_logdir.py"
    output = subprocess.check_output(cmd, shell=True).strip()

    tracingFiles = []

    # Find the tracing files of pathPlanning
    for file in os.listdir(output):

        # Only look at trace_* files
        if file.startswith("trace_"):

            # Build filepath
            filepath = os.path.join(output, file)
            with open(filepath, "r") as f:
                for line in f:

                    # If the file is a pathPlanning tracing file, "pathPlanning" is in the first line.
                    if 'pathPlanning' not in line:
                        # This file is NOT pathPlanning tracing
                        # Skip.
                        break

                    # If this line is reached, file IS pathPlanning tracing
                    tracingFiles.append(filepath)
                    break

    deltas = []

    for tracingFile in tracingFiles:
        cmd = "grep -E 'cAbstractPathPlanning|main_ros' %s | grep -E 'iteratePP  >|execute  <'" % tracingFile
        output = subprocess.check_output(cmd, shell=True).strip()

        for line in output.split("\n"):
            # distinguish between start and end times
            if "iteratePP  >" in line:
                # save start time
                values = line.split(' ')
                startTime = datetime.strptime(values[0], "%Y-%m-%d,%H:%M:%S.%f")
            else:
                # save end time
                values = line.split(' ')
                endTime = datetime.strptime(values[0], "%Y-%m-%d,%H:%M:%S.%f")

                # print delta
                delta = endTime - startTime
                #print delta.microseconds, "microseconds ==", delta.microseconds/1000.0, "milliseconds"

                deltas.append(delta.microseconds)


    # Compute average delta
    totalMicroSec = 0
    for delta in deltas:
        totalMicroSec += delta
    print "Top 10 (in microseconds): %s" % str(sorted(deltas, reverse=True)[0:10])
    print "Average:", totalMicroSec / len(deltas), "microseconds ==", totalMicroSec / len(deltas) / 1000.0, "milliseconds"
    print "1000 microseconds = 1 millisecond"
