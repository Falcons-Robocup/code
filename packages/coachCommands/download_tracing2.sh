#!/bin/bash
#
# JFEI 2015-03-31 creation
#
# download tracing from robot to local falcons_control tmpdir

echo "still not always working ! "
echo "try ~/turtle/scripts/downloadtrace"
exit 1

robotnum=$1
if [ -z "$robotnum" ]; then
   echo "ERROR: provide robotnum"
   exit 1
fi
echo "downloading tracing from robot $robotnum ..." 
logdir=`$TURTLEROOT/packages/coachCommands/newest_logdir.py`
sudo mkdir -p $logdir
sudo chmod 0777 $logdir
scp $robotnum:$logdir/* $logdir

