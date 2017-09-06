#!/bin/bash
#
# JFEI 2015-03-31 creation
# MKOE 2015-04-11 fix user login
#
# download tracing from robot to local falcons_control tmpdir

robotnum=$1
if [ -z "$robotnum" ]; then
   echo "ERROR: provide robotnum"
   exit 1
fi
echo "downloading tracing from robot$robotnum ..." 
logdir=`newest_logdir.py`
sshpass -p Robocup scp robocup@$ROBOTNET$robotnum:$logdir/* $logdir


