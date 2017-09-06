#!/bin/bash
#
# falconsbuild.sh
# Top level build-all wrapper around rosmake. 
#
# 




rosmake $* `package_list` || exit 1

# force FS sync
sync

# clear image state
echo > `imageStatusFile`

