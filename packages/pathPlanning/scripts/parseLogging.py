""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import sys, subprocess, os
from datetime import datetime

spacingStr = '   '
verbose = False

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

    # Create output directory in pathPlanning/scripts if not exists
    tracingOutput = "/home/robocup/falcons/code/packages/pathPlanning/scripts/tracingOutput"
    if not os.path.exists(tracingOutput):
        os.makedirs(tracingOutput)

    # Parse each file
    fileNr = 0
    for tracingFile in tracingFiles:
    
        if verbose:
            print "Parsing %s..." % tracingFile
        
        #Maintain a function "depth"
        depth = 0

        # Keep all lines up to FILENAME:LINE in a list to grab the maximum length when done.
        prefixes = []
        msgs = []

        with open(tracingFile, "r") as f:
            for line in f:

                line = line.strip()
                if line == "":
                    continue

                #TIME THREAD FILENAME:LINE > FUNCTION1
                #TIME THREAD FILENAME:LINE   > FUNCTION2
                #TIME THREAD FILENAME:LINE   < FUNCTION2
                #TIME THREAD FILENAME:LINE < FUNCTION1

                #parse time
                values = line.split(' ')
                time = datetime.strptime(values[0], "%Y-%m-%d,%H:%M:%S.%f")

                #parse thread
                thread = values[1]

                #parse filename
                filename = values[2].split('/')[-1]

                #parse remaining message
                msg = ' '.join(values[3:]).strip()

                #parse opening function
                if '  >' in msg:
                    # Move the > to the front
                    msg = '> ' + msg.replace('  >',' :')

                    # Add depth spacing to the msg
                    for i in range(depth):
                        msg = spacingStr + msg

                    # Increase depth for next command
                    depth += 1

                #parse closing function
                elif '  <' in msg:
                    # Move the < to the front
                    msg = '< ' + msg.replace('  <',' :')

                    # Increase depth for next command
                    depth -= 1

                    # Add depth spacing to the msg
                    for i in range(depth):
                        msg = spacingStr + msg

                #no opening function nor closing function
                else:
                    # Add depth spacing to the msg
                    for i in range(depth):
                        msg = spacingStr + msg

                newline = "%s %s %s" % (time, thread, filename)
                prefixes.append(newline)
                msgs.append(msg)

        # Now compute the longest prefix
        longestPrefix = 0
        for prefix in prefixes:
            if len(prefix) > longestPrefix:
                longestPrefix = len(prefix)

        # Keep some space between longest prefix and msg
        longestPrefix += 2

        # Now print everything to the new file
        outputFile = os.path.join(tracingOutput, str(fileNr) + ".txt")
        if verbose:
            print "Writing to %s..." % outputFile
        with open(outputFile, "w") as f:

            for j in range(len(prefixes)):

                prefix = prefixes[j]
                msg = msgs[j]

                # Make sure the prefix has the same length as the longest prefix
                while len(prefix) < longestPrefix:
                    prefix += " "

                line = prefix + msg + "\n"
                f.write(line)

        fileNr += 1

    # Wrote all to tracingOutput
    # Open all with nedit
    traceFiles = []
    for file in os.listdir(tracingOutput):
        traceFiles.append(os.path.join(tracingOutput, file))
    cmd = "nedit " + " ".join(traceFiles)
    os.system(cmd)
