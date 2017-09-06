#!/bin/bash
#
# Used in control suite.
# Kill processes on current [real] robot. 
# This is needed because commands stay alive when ssh connections are severed...
# I tried to work around this in sshwrapper but failed. 
# So call this script remotely at the right time.
#
# JFEI 2015-02-10 creation
# MKOE 2015-04-09 also kill the peripherals.py and visualizer.py
# JFEI 2015-05-05 added some more
# JFEI 2015-05-16 make silent
# JFEI 2015-05-30 graceful logger shutdown (prevent active .bag); 
#                 option to not kill falcons_control since it can be parent
# JFEI 2015-06-07 add option to list processes rather than killing them
# JFEI 2015-06-13 add option to keep roscore

# TODO
# * 
# 



# debug
#date '+%Y%m%d_%H%M%S.%N'
#echo $0 $*


# argument parsing
KILL=1
LIST=0
CONTROL=1
ROSCORE=1
LOGGER=1
while [[ $# > 0 ]]
do
    key="$1"
    case $key in
        keepcontrol)
        CONTROL=0
        ;;
        keeproscore)
        ROSCORE=0
        ;;
        list)
        LIST=1
        KILL=0
        ;;
        *)
                # unknown option
        ;;
    esac
    shift # past argument or value
done


# more debugging: list before kill
#if [ "$LIST" = 0 ]; then
#    falcons_kill.sh list
#fi


# LIST option: show details for a process id
sub_detail_pid() {
    for pid in $*; do
        ps h $pid
    done
}

# wrapper around pkill with optional extra details
sub_pkill() {
    pids=`pgrep -f $1`
    if [ "$LIST" = 1 ]; then
        echo "debug: sub_pkill $1"
        sub_detail_pid $pids
    fi
    if [ "$KILL" = 1 ]; then
        for pid in $pids; do
            kill -9 $pid
        done >/dev/null 2>&1
    fi
}

# search for processes matching string and kill it (or them)
sub_strkill() {
    pids=`ps -ef | grep $1 | grep -v 'grep' | awk '{print $2}'`
    if [ "$LIST" = 1 ]; then
        echo "debug: sub_strkill $1"
        sub_detail_pid $pids
    fi
    if [ "$KILL" = 1 ]; then
        if [ ! -z "$pids" ]; then
            echo $pids | xargs kill -9 >/dev/null 2>&1
        fi
    fi
}

# kill all relevant processes
sub_killall() {

    sub_pkill rosrun
    sub_pkill motionHAL
    sub_pkill peripheralsInterface
    sub_pkill teamplay
    sub_pkill Node
    sub_pkill worldModel
    sub_pkill WorldModel
    sub_pkill simteam
    sub_pkill simball
    sub_pkill main_logger
    sub_pkill setrobotspeed
    sub_pkill shoot_
    sub_pkill rostopic
    sub_pkill visualize
    sub_pkill simulator
    sub_pkill refboxRelay
    
    sub_strkill peripherals_sim.py
    sub_strkill peripherals.py
    sub_strkill ref_box_listener
    sub_strkill detectBall

    if [ "$ROSCORE" = 1 ]; then
        sub_strkill roscore
        sub_strkill rosmaster
        sub_strkill rosout
    fi

    sub_strkill matchlog
    #sub_strkill coach
    sub_strkill rosnode
    sub_strkill diagnostics
  
    if [ "$CONTROL" = 1 ]; then
        sub_pkill falcons_control
    fi

    #sub_pkill wrapp
}

# first kill the logger, to prevent active .bag file
if [ $LOGGER = 1 ]; then
    # TODO LIST mode
    if [ $KILL = 1 ]; then
        (
            recordnode=`rosnode list | grep record`
            if [ -n "$recordnode" ]; then
                rosnode kill $recordnode
            fi
        ) > /dev/null 2>&1
    fi
fi




# do the kill
sub_killall


# reset robot status file
if [ "$SIMULATED" = 0 ]; then
     cat > $ROBOT_STATUS_FILE <<EOF
online=false
claimedby=none
EOF
fi

#[ -f /var/tmp/tracing_trigger ] && rm /var/tmp/tracing_trigger
# do not remove tracing trigger: falcons_control.py manages its existence by itself

exit 0

