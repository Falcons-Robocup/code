#!/bin/bash
#
# After each falcons_control session a .bag file is created with all gathered data.
# In real mode this is always stored centrally. This script is called
# at the end of the session to put the file on a central location.
#
# 
# JFEI 2015-07-17 creation

#
# TODO
# * 
# 


exit 0 # JFEI disabled for now, network is piss poor ...



targethost=butter
targetdir=/home/robocup/LOGS
sourcehostname=`hostname`
sourcedir=`newest_logdir.py`
sessionid=`newest_logdir.py | sed 's/.*control_//'`
sourcebag=`ls -1 $sourcedir/*.bag`


# auxiliary function to ask user to store the file manually
sub_error() {
    echo "ERROR: $1"
    echo "could not store bagfile $sourcebag"
    echo "please do this manually - align with JFEI!!!"
    exit 0 # to not generate exception in falcons_control.py
}


# first check if targethost is available
ping -W 1 -c 1 $targethost >/dev/null 2>/dev/null
if [ "$?" != "0" ]; then
    echo "check if your network is the same as the one from hippie (Jan)"
    sub_error "could not ping targethost=$targethost"
fi

# determine target name
targetid=`printf "%s_%s" $sessionid $sourcehostname`

# copy
scpcmd="scp $sourcebag $targethost:$targetdir/$targetid.bag"
echo
echo "storing logging..."
echo "  $scpcmd"
$scpcmd
if [ "$?" != "0" ]; then
    sub_error "copy failed"
fi
   
   



