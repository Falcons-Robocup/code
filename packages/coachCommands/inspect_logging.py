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
import datetime, time
from fnmatch import fnmatch
from newest_logdir import newest_logdir



if __name__ == '__main__':
   # Argument parsing.
   parser     = argparse.ArgumentParser(description='falcons logging inspection')
   parser.add_argument('--logdir', help='logging dir', default=newest_logdir())
   parser.add_argument('--filemask', help='file mask', default=["*.txt"])
   args       = parser.parse_args()
   # Find nonempty files
   if args.logdir == None:
      raise Exception('could not find latest logging dir, give it explicitly via --logdir')
   # Add /var/tmp if not given
   if not os.path.isdir(args.logdir):
      args.logdir = "/var/tmp/" + args.logdir
   if not os.path.isdir(args.logdir):
      raise Exception('could not find logdir')
   filelist   = []
   for dirname, dirnames, filenames in os.walk(args.logdir):
      for f in filenames:
         filename = dirname + "/" + f
         # only nonempty files
         if os.stat(filename).st_size != 0:
            # check if this file matches on of the given masks
            for filemask in args.filemask:  
               if fnmatch(f, filemask):
                  filelist.append(filename)
                  break
   # Sort to get some kind of consistency
   filelist.sort()
   # Open editor (use nedit)
   os.system("nedit " + " ".join(filelist))

