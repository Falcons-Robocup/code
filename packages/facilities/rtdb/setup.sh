#!/bin/bash
# setup everything required to make rtdb work


# exit upon any error
set -e

# offer a mode to simply check, which is useful for wtf script
checkMode=0
if [ "$1" = "-v" ]; then
    checkMode=1
fi

# settings and preparation
tmpDir=/var/tmp/rtdbSetup
if [ -d $tmpDir ]; then
    rm -rf $tmpDir
fi
mkdir $tmpDir

# cleanup
if [ -d $tmpDir ]; then
    rm -rf $tmpDir
fi
if [ $checkMode = 0 ]; then
    echo "everything should be OK now, please verify with:"
    echo "   fmake -s rtdb2"
fi

