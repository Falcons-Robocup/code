#!/bin/bash
#
# setupEnv.sh
# Hook in .bashrc to set common paths, aliases etc.
# Previously known as falconsconfig.sh
#
# See also: setupEnvBashrcHook.sh
# Or do it manually/customize: add at the bottom of .bashrc:
#
#      source /home/robocup/falcons/code/scripts/setupEnv.sh
#




# setup path specific to falcons code/repo
FALCONS_SCRIPTS_PATH="$( cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 ; pwd -P )"
source $FALCONS_SCRIPTS_PATH/setupEnvPaths.sh

# setup system paths
# Use the Atmel version of avr-gcc because the .deb version results linking error that section .BOOT overlaps with section .data
export PATH=$PATH:/opt/avr_v3.5.0/bin
export PATH=$PATH:/sbin


# TODO: revise old TURTLE name usage, maybe instead use generic AGENT (like Cambada)?

# Set TURTLE5K_ROBOTNUMBER if on robot
# (no need to do this via .bashrc)
export SIMULATED=0
robotnum=`hostname | grep FALCON | sed 's/FALCON-//'`
if [ ! -z "$robotnum" ]; then
    export TURTLE5K_ROBOTNUMBER=$robotnum
    export AGENT=$robotnum # rtdb tools traditionally use a more general environment variable name
fi

# Set TURTLE5K_TEAMNAME
# Note that falcons_control.py can override this to be teamB
export TURTLE5K_TEAMNAME=teamA

# to prevent the following warning: what():  locale::facet::_S_create_c_locale name not valid
export LC_ALL=C

# aliases and handy shortcuts
source "$FALCONS_SCRIPTS_PATH/alias"
fcd() {
    if [ -z "$1" ]; then
        cd $FALCONS_CODE_PATH
    else
        d=`fdir $1`
        for t in $d $FALCONS_CODE_PATH/$d ; do
            if [ -d $t ]; then
                cd $t
                break
            fi
        done
    fi
}

# Colorful Bash prompt that indicates git branch
source $FALCONS_SCRIPTS_PATH/git-prompt.sh
PS1='\[\033[01;32m\]\u@\h\[\033[01;31m\] [$(__git_ps1 "%s" )] \[\033[01;34m\]\W\[\033[00m\] \$ '


