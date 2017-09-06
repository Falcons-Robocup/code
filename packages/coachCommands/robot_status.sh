#!/bin/bash
#
# JFEI 2015-03-22 creation
#
# script called from falcons_control.py
# to be executed on robot, send back output
# to look for critical issues on robot, e.g.
#  * dmesg logging shows 'safety check'
#  * output logging shows ROS errors during normal operation
#  * ... anything we want to be notified of


if [ -z "$1" ]; then
   echo "missing argument: logdir"
   exit 1
fi

if [ ! -d "$1" ]; then
   echo "logdir not found"
   exit 1
fi

grep -i error $1/*.std* | grep -v "Ignored obstacle" | tail -4
dmesg | grep safety
grep -i tty $1/*.std* 

# module activity flags
rosnamespace=/${TURTLE5K_TEAMNAME}/robot${TURTLE5K_ROBOTNUMBER}
for module in actionhandler shootplanning pathplanning; do
   srv=$rosnamespace/s_${module}_get_active
   response=`rosservice call $srv | egrep -o 'True|False'` 
   printf "%20s : %s\n" $module $response
done

# worldmodel active robot list
echo -n "active robots according to WorldModel: "
rosservice call $rosnamespace/s_get_active_robots | sed 's/active_robots://'

