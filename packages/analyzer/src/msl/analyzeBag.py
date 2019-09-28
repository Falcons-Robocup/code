""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Analyze a Falcons logging file (.bag).
# Command-line entry point.
#
# FALCONS // Jan Feitsma, August 2017



# python includes
import os
import argparse

# package includes
from datastreamBag import dataStreamBag
from analyzer import analyzer
import settings



# extract statistics from single file
def main(bagfile, verboseLevel=0, sampleFrequency=5.0):
    # load the file as data stream
    d = dataStreamBag()
    d.setVerboseLevel(verboseLevel)
    d.load(bagfile)
    d.fuse(sampleFrequency)
    # analyze it
    a = analyzer(d)
    a.setVerboseLevel(verboseLevel)
    a.runFull()
    s = a.statistics
    # we do not have a data stream for the other team
    s.teams[1].setNoData() 
    return s


# command line interface
if __name__ == '__main__':

    # argument parsing
    parser     = argparse.ArgumentParser(description='analyze a Falcons .bag logging file')
    parser.add_argument('-f', '--frequency', help='sample frequency', default=settings.DEFAULT_SAMPLE_RATE)
    parser.add_argument('-v', '--verbose', help='verbose', action='store_true')
    parser.add_argument('-V', '--verboselevel', help='verbose level', type=int, default=0)
    parser.add_argument('logfile', help='Falcons .bag log file', default=None)
    args       = parser.parse_args()
    
    # analyze
    if not os.path.isfile(args.logfile):
        raise Exception("file not found: {}".format(args.logfile))
    if args.verbose:
        args.verboselevel = 1
    stats = main(args.logfile, args.verboselevel, args.frequency)
    
    # display results
    print stats

