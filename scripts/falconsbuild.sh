#!/bin/bash
#
# falconsbuild.sh
# 



fmake -a $*
retval=$?

if [ "$retval" != 0 ]; then
    exit 1
fi

# force FS sync
# (we had in the past sometimes empty targets when a build was interrupted / robot lost power)
sync

