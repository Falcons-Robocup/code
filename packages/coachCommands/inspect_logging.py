# Copyright 2015-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
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

