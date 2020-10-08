#!/bin/bash



# file to be modified
targetfile=~/.bashrc

# legacy check for falconsconfig
if grep -q falconsconfig $targetfile ; then
    echo "ERROR: .bashrc contains a reference to old falconsconfig, please remove it manually"
    exit 1
fi

separatorline="# Falcons setup hook"
hookline="source ~/falcons/code/scripts/setupEnv.sh"

# note: /home/robocup/falcons/code is the default configuration
# but this can be customized

# first time setup: append
if ! grep -q "$separatorline" $targetfile ; then
    echo "$separatorline" >> $targetfile
    echo "$hookline" >> $targetfile
    echo "" >> $targetfile
else
    echo "WARNING: .bashrc already contains a reference to setupEnv, skipping"
fi
