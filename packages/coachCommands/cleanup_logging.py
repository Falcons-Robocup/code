""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Cleanup logging directories on robot and devlt.



import argparse
import sys, os, string
import datetime, time
import shutil
from bcolors import bcolors

def cleanup_logging(n = 3):
   # cleanup oldest directories, only leave newest few
   # create dict with timestamps as key and dirname as value
   logdirs = {}
   for dirname, dirnames, filenames in os.walk('/var/tmp'):
      for subdirname in dirnames:
         if "falcons_control" in subdirname:
            t = os.path.getmtime(dirname + '/' + subdirname)
            logdirs[t] = subdirname
   # remove oldest ones
   count = 0
   for key in reversed(sorted(logdirs.iterkeys())):
      count += 1
      if count > n:
         dirname = '/var/tmp/' + logdirs[key]
         # JFEI 2015-05-14 remove useless message
         #print bcolors.OKGREEN + "cleaning up old logging dir: " + dirname + bcolors.ENDC
         shutil.rmtree(dirname)


if __name__ == '__main__':
   # Argument parsing.
   parser     = argparse.ArgumentParser(description='falcons logging cleanup')
   parser.add_argument('-n', '--num', type=int, help='no visualizer', default=30)
   args       = parser.parse_args()
   # cleanup log folders
   cleanup_logging(args.num)
   # cleanup tracing trigger -- no, this is supposed to be managed by falcons_control.py
   #if os.path.isfile("/var/tmp/tracing_trigger"):
      #os.system("rm /var/tmp/tracing_trigger")

