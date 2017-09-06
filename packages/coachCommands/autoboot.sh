#!/bin/bash
#
# If required, automatically boot the software on real robot.
# This is useful during match mode to get robot back in play after a reboot
# without manual commands, as this is not allowed.
# See also ticket #234.
# 
# This script is called from Ubuntu "Startup applications".
# Instructions to enable it:
#  * do for each robot r1..r6 via TeamViewer:
#    * click on "dash home" top-left
#    * type in the search field "startup"
#    * open "Startup Applications"
#    * add a new entry
#      * Name   : falconsControl autoboot
#      * Command: /home/robocup/falcons/code/packages/coachCommands/autoboot.sh
#      * Comment: falconsControl autoboot
#
# This script will actually only autoboot if it finds the string    
#    mode=match
# in ~/robot.cfg (environment $ROBOT_CONFIG_FILE)
# 
# JFEI 2015-07-10 creation

#
# TODO
# * 
# 



# create standard environment
source /home/robocup/falcons/code/scripts/falconsconfig.sh

if [ -f $ROBOT_CONFIG_FILE ]; then
    if grep -q "mode=match" $ROBOT_CONFIG_FILE ; then
    
        sleep 15 # give some time to come online

        # workaround for M4log issue (#268)
        m4logload=`top -b -n 1 | grep m4log | awk '{print $9}'`
        m4logkilledfile=/var/tmp/m4logkilled
        [ -f $m4logkilledfile ] && rm $m4logkilledfile
        if [ ! -z "$m4logload" ]; then
            if [ "$m4logload" -gt "40" ]; then
                pkill m4log
                touch $m4logkilledfile
            fi
        fi

        # proceed with autoboot
        # first determine logdir to capture all output
        LOGDIR="/var/tmp/falcons_control_"`date +%Y%m%d_%H%M%S`
        # set command string
        cmd="outputwrapper2 $LOGDIR falcons_control.py -r --logdir $LOGDIR"
        # now determine whether or not to start in teamB mode
        if grep -q "team=teamB" $ROBOT_CONFIG_FILE ; then
            cmd="$cmd --teamB"
        fi
        # set post-init demo (invented for techChallenge Hefei)
        postinitdemo=`get_robot_config postinitdemo`
        if [ ! -z "$postinitdemo" ]; then
            if [ "$postinitdemo" != "none" ]; then
                cmd="$cmd --postinitdemo $postinitdemo"
            fi
        fi
        # generic extra options (TODO: make nicer, now it is a bit overlapping/contradicting with above options...)
        bootopts=`get_robot_config bootopts 2>/dev/null`
        if [ "$?" = "0" ]; then
            if [ ! -z "$bootopts" ]; then
                cmd="$cmd $bootopts"
            fi
        fi
        # run the command in foreground
        $cmd
    fi
fi



