""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# 2016-07-12 JFEI creation
#
# Description: visualize a bag file.
#  * spawn a small playbackControl window, which provides a slider and a few buttons to browse
#  * spawn a roscore
#  * dump data on topics
#  * spawn a visualizer to draw the data
#
# 


import sys,os,time
import argparse
import threading
import traceback
from playbackControl import PlaybackControl
from matchlog import MatchLogBaggedPublisher
from messageUpgrader import MessageUpgrader


# globals
gPbCtrl = None
gPbGuiTr = None



def cleanup():
    # processStop is not needed apparently (?)
    #os.system('processStop roscore')
    #os.system('processStop visualizer')
    gPbCtrl.quit()


if __name__ == '__main__':
    # Argument parsing.
    parser     = argparse.ArgumentParser(description='falcons matchlog viewer')
    parser.add_argument('bagfile', help='bag file of the log', nargs='?', default=None)
    args       = parser.parse_args()

    # Spawn a roscore
    os.system('processStart roscore &')

    try:

        # Construct MatchLog object 
        matchlog   = MatchLogBaggedPublisher(args.bagfile, MessageUpgrader())
        tStart     = matchlog.tStart
        tEnd       = matchlog.tEnd
        tElapsed   = matchlog.tElapsed
        # Override options - TODO


        # Construct the playback window and run its GUI loop in a dedicated thread
        gPbCtrl    = PlaybackControl(tStart, tEnd, tElapsed)
        gPbGuiTr   = threading.Thread(target=gPbCtrl.run)
        gPbGuiTr.start()
        
        # Spawn the new visualizer
        os.system('processStart visualizer &')
        
        # Run the matchlog data handler loop to start publishing
        matchlog.run(gPbCtrl)
        
    except:
        print "Unexpected error:", sys.exc_info()[0]
        print traceback.format_exc()
        cleanup()
        raise

    # All went well - cleanup
    cleanup()



