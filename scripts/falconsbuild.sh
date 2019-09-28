#!/bin/bash
#
# falconsbuild.sh
# Top level build-all wrapper around rosmake. 
#
# 




rosmake $* `package_list` 
retval=$?

# only on buildserver: keep track of state and post a comment to commit in Gitlab when state changes
processBuildServerResult FULLBUILD $retval

if [ "$retval" != 0 ]; then
    exit 1
fi

# force FS sync
sync

# clear image state
echo > `imageStatusFile`

