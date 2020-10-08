#!/usr/bin/env python3
#
# Inspect logging directory.




import os

BASE = '/var/tmp'

def newest_logdir():
   output = None
   prevt = None
   for dirname, dirnames, filenames in os.walk(BASE):
      for subdirname in dirnames:
         if "falcons_control" in subdirname and dirname == '/var/tmp':
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

if __name__ == '__main__':
   print(newest_logdir())

