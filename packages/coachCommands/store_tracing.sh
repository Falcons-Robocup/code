#!/bin/bash
#
# Download all tracing from robots, so it can be removed and the disk does not fill up.
# 
# JFEI 2015-07-19 creation

#
# TODO
# * 
# 





targethost=butter
targetdir=/home/robocup/TRACE


# check targethost
ping -W 1 -c 1 $targethost >/dev/null 2>/dev/null
if [ "$?" != "0" ]; then
    echo "ERROR: targethost $targethost not reachable"
    exit 1
fi

# construct list of robots to apply to
robots=""
while :; do
    # check if it is a number
    found=0
    for irobot in `seq 1 6`; do
        if [ "$1" = "$irobot" -o "$1" = "r$irobot" ]; then
            robots="$robots r$irobot"
            found=1
        fi
    done
    if [ "$found" = "1" ]; then
        shift
    else
        break
    fi
done
if [ -z "$robots" ]; then
    # none specified = all
    robots="r1 r2 r3 r4 r5 r6"
fi


# run the command
for robot in $robots; do
    ping -W 1 -c 1 $robot >/dev/null 2>/dev/null
    if [ "$?" == "0" ]
    then
        scp -r "$robot:/var/tmp/falc*" $targethost:$targetdir/$targetid/$robot
        # remove to save diskspace
        echo "SCP done on $robot, clearing its /var/tmp/falc*"
        ssh $robot "rm -rf /var/tmp/falc*"
    else
        echo "ERROR: could not reach $robot"
    fi
done


