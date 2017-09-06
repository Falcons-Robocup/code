#!/bin/bash
#
# JFEI 2015-03-21 creation
#
# create a bagfile on the matchlog topic
# 


if [ -z "$1" ]; then
    # just take latest one
    logdir=`newest_logdir.py`
else
    logdir=$1
fi

if [ ! -d "$logdir" ]; then
    echo "logdir not found"
    exit 1
fi

topics="/teamA/robot./g_diag_.*|/teamA/g_worldmodel_team|/teamA/g_ana_.*|/teamA/g_diag_.*|/teamA/robot./g_ana_.*"
rosbag record -e $topics -o $logdir/


