""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Inspect logging directory.



import argparse
import sys, os, string
import signal
import datetime, time
from fnmatch import fnmatch
from newest_logdir import newest_logdir


# nicely cleanup processes on interrupt, prevent restartWrapper from starting them again
def signal_handler(signal, frame):
    os.system('processStopAll')
    sys.exit(0)



if __name__ == '__main__':
    # Argument parsing.
    parser     = argparse.ArgumentParser(description='falcons matchlog inspection')
    parser.add_argument('--old', help='use old visualizer', action='store_true')
    parser.add_argument('--logdir', help='logging dir', default=None)
    parser.add_argument('--zoom', help='zoom factor for screen', default='auto')
    parser.add_argument('bagfile', help='bag file to use', nargs='?', default=None)
    args       = parser.parse_args()
    # In case bag file is provided, then use it
    # NOTE: previously there was a piece of code here to fix improperly closed
    #       bag files, but proper closing is now achieved in falcons_control.py/falcons_kill.sh
    if args.bagfile != None:
        bagfile = args.bagfile
    else:
        if args.logdir == None:
            args.logdir = newest_logdir()
        # See which .bag files there are 
        bagfiles = []
        actfiles = []
        for f in os.listdir(args.logdir):
            if fnmatch(f, '*.bag'):
                bagfiles.append(args.logdir + '/' + f)
        assert(len(bagfiles) == 1)
        bagfile = bagfiles[0]
    # Execute the visualizer on bagfile
    signal.signal(signal.SIGINT, signal_handler)
    if args.old:
        os.system('$TURTLEROOT/packages/visualizer/src/oldpy/visualize_matchlog.py' + ' --zoom ' + str(args.zoom) + ' ' + bagfile)
    else:
        os.system('$TURTLEROOT/packages/visualizer/src/playbackMatchlog.py ' + bagfile)
    os.system('processStopAll')

