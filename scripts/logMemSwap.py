#!/usr/bin/env python3

import sys, os, signal
import traceback
import time
from subprocess import call

def newest_logdir():
   output = None
   prevt = None
   for dirname, dirnames, filenames in os.walk('/var/tmp'):
      for subdirname in dirnames:
         if "falcons_control" in subdirname:
            d = '/var/tmp/' + subdirname
            t = os.path.getmtime(d)
            if prevt != None:
               if t > prevt:
                  output = d # overwrite until newest one is stored
                  prevt = t
            else:
               output = d # overwrite until newest one is stored
               prevt = t
   return output

logDir = newest_logdir()
print(logDir)
while(True):
    os.system("smem >> "+ str(logDir) + "/memSwap.txt")
    os.system("echo ============================ >> "+ (logDir) + "/memSwap.txt")
    time.sleep(5)
