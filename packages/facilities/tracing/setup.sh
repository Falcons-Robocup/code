#!/bin/bash
# setup everything required to make tracing work


# exit upon any error
set -e

# offer a mode to simply check, which is useful for wtf script
checkMode=0
if [ "$1" = "-v" ]; then
    checkMode=1
fi

# data repo up to date?
srcLib=~/falcons/data/external/tracing-framework/lib/libwtf.so
if [ ! -f $srcLib ]; then
    if [ $checkMode = 1 ]; then
        echo "data repo is not up-to-date"
        exit 1
    else
        echo "updating data repo ..."
        cd ~/falcons/data
        git pull || exit 1
    fi
fi
if [ $checkMode = 0 ]; then
    echo "everything should be OK now, please verify with:"
    echo "   fmake tracing"
fi

