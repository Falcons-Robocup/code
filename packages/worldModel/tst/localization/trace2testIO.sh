#!/usr/bin/env bash
#
# 2017-07-09 JFEI
# utility to extract localization raw data and algorithm result from given trace file
# can be used in test cases


# which tracefiles?
traceFiles="$*"
if [ -z "$traceFiles" ]; then
    cd `ls -1dt /var/tmp/falco* | head -1`
    traceFiles=`ls -altr1 trace_*wm*.txt | tail -3 | awk '{print $NF}'`
fi

# filter tracing
grep robotLocalization.cpp $traceFiles \
    | egrep "INITIALIZE|CALCULATE|ENCODER|VISION" \
    | sed 's/\(.*\)INITIALIZE/INITIALIZE/' \
    | sed 's/\(.*\)CALCULATE/CALCULATE/' \
    | sed 's/\(.*\)ENCODER/ENCODER/' \
    | sed 's/\(.*\)VISION/VISION/'
grep robotLocalization.cpp $traceFiles \
    | grep "RESULT" \
    | sed 's/\(.*\)RESULT/RESULT/'

