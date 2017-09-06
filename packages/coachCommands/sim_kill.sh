#!/bin/sh
#
# usage: to kill all rosnodes belonging to a given namespace
#
# 20150411 MKOE first release to repos
#
#
SCRIPTNAME=`basename $0`
if [ $# -ne 1 ]
then
   echo "Usage: $SCRIPTNAME /teamX/robotN  (where X=A or B and N=1-5)"
   exit
fi

for NODE in `rosnode list | grep "$1" `
do
  echo "Killing node: $NODE"
  rosnode kill $NODE >/dev/null 2>/dev/null
done

