#!/bin/bash
#
# falconsconfig.sh
# Hook in .bashrc to set common paths, aliases etc.
#
# 
# Users should only add the following at the bottom of .bashrc:
# 
#      source /home/robocup/falcons/code/falconsconfig.sh
#



# to prevent the following warning: what():  locale::facet::_S_create_c_locale name not valid
export LC_ALL=C


# ROS environments
source /opt/ros/kinetic/setup.bash

# default codebase root directory
if [ -z "$TURTLEROOT" ]; then
    if [ -d $HOME/falcons/code ]; then
        export TURTLEROOT=$HOME/falcons/code
    else
        echo "ERROR: could not find ~/falcons/code GIT workspace ... WTF?"
    fi
fi

# ROS packages lookup dirs
export ROS_PACKAGE_PATH=$TURTLEROOT/packages:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$TURTLEROOT/externals:$ROS_PACKAGE_PATH

# Standard commandline utilities
export PATH=$PATH:/sbin
export PATH=$TURTLEROOT/scripts:$PATH
export PATH=$TURTLEROOT/packages/coachCommands:$PATH
export PATH=$TURTLEROOT/packages/processManager:$PATH
export PATH=$TURTLEROOT/packages/jobManager:$PATH
export PATH=$TURTLEROOT/packages/robotControl:$PATH

# Use the Atmel version of avr-gcc because the .deb version results linking error that section .BOOT overlaps with section .data
export PATH=/opt/avr_v3.5.0/bin:$PATH

# Add path to own Robot Framework Libraries
export PYTHONPATH=$TURTLEROOT/testing/execution/lib:$PYTHONPATH
export PYTHONPATH=/opt/ros/groovy/lib/python2.7/dist-packages:$PYTHONPATH
# Add path to python module to access C++ field environment library
export PYTHONPATH=$PYTHONPATH:$TURTLEROOT/packages/facilities/environment/pymodule

# standard aliases
source "$TURTLEROOT/scripts/alias"


# Set the robot network, if necessary
# default (see http://timmel.no-ip.biz:8000/wiki/MSLNetworkSetup)
export ROBOTNET=172.16.74.5
# fallback for RobocupASML router
ifconfig -a | grep -q 192.168.5 && export ROBOTNET=192.168.5.5
ifconfig -a | grep -q 10.0.0 && export ROBOTNET=10.0.0.
# TODO robotnet 10.0.0.1 in case of wired? or via falcons_control -e? 

# Set TURTLE5K_ROBOTNUMBER if on robot
# (no need to do this via .bashrc)
export SIMULATED=0
robotnum=`hostname | grep FALCON | sed 's/FALCON-//'`
if [ ! -z "$robotnum" ]; then
    export TURTLE5K_ROBOTNUMBER=$robotnum
fi

# Set TURTLE5K_TEAMNAME
# Note that falcons_control.py can override this to be teamB
export TURTLE5K_TEAMNAME=teamA

# Cleanup automatically generated logging
# for buildserver, allow a fairly large limit, to be able to inspect why test runs are failing
if [ `hostname` = buildserver-jade ]; then
    cleanup_logging.py -n 100
fi

# aliases and handy shortcuts
ns() {
    echo "Setting ROS_NAMESPACE to : /$1"
    export ROS_NAMESPACE="/$1"
}

# Colorful Bash prompt that indicates git branch
source $TURTLEROOT/scripts/git-prompt.sh
PS1='\[\033[01;32m\]\u@\h\[\033[01;31m\] [$(__git_ps1 "%s" )] \[\033[01;34m\]\W\[\033[00m\] \$ '


