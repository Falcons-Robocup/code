""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import sys, os, subprocess, shlex

if __name__ == "__main__":

    if len(sys.argv) == 1:
        print "Usage: python profiler.py <process> <-t>"
        print "Runs strace on each thread that belongs to the given process. (e.g. pathplanningNode )"
        print "Will print all outputs to /var/tmp/strace/<process>/<pid>.out"
        print "Optionally use -t to only compute totals instead of per thread."
        exit()
    else:
        # Find all threads belonging to given process
        output = ""
        try:
            cmd = "ps -efL | grep %s | grep -v outputwrapper | grep -v python | grep -v grep" % sys.argv[1]
            output = subprocess.check_output(cmd, shell=True)
        except:
            print "Failed to find process '%s' when calling:" % sys.argv[1]
            print cmd
            exit()


        # Check if output directory exists
        if not os.path.exists("/var/tmp/strace"):
            os.makedirs("/var/tmp/strace")
        if not os.path.exists("/var/tmp/strace/%s" % sys.argv[1]):
            os.makedirs("/var/tmp/strace/%s" % sys.argv[1])

        # Build strace commands for all applicable process ids
        commands = []

        if len(sys.argv) > 2:
            # Build first command for total process
            pid = shlex.split(output.strip())[1]
            outputTotal = "/var/tmp/strace/%s/total.out" % sys.argv[1]
            cmd = "strace -c -S time -f -o %s -p %s" % (outputTotal, pid)
            commands.append(cmd)

        else:
            # Build commands for separate threads
            for line in output.strip().split('\n'):
                threadId = shlex.split(line)[3]
                outputFile = "/var/tmp/strace/%s/%s.out" % (sys.argv[1], threadId)

                cmd = "strace -c -S time -o %s -p %s" % (outputFile, threadId)
                commands.append(cmd)

        # run all strace commands in parallel
        processes = [subprocess.Popen(cmd, shell=True) for cmd in commands]

        print "strace profiling running for process '%s'." % sys.argv[1]
        print "Hit CTRL+C to stop profiling..."

        # wait for completion
        for p in processes: p.wait()
