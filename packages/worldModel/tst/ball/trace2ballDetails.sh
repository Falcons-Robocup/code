#!/usr/bin/env bash
#
# 2016-11-13 JFEI
# utility to extract ball tracker details from given trace file
# output is comparable to Matlab tracing (fbt_trackers_info.m)
# this is useful for debugging ported implementation


# header line
echo "#           time #tr #b   id conf   solx   soly   solz  solvx  solvy  solvz   age #meas ncam omni qcam qage   qm  qfr   qz   qv qfit  res"

# which tracefiles?
traceFiles="$*"
if [ -z "$traceFiles" ]; then
    cd `ls -1dt /var/tmp/falco* | head -1`
    traceFiles=`ls -altr1 trace_A0_unknown*.txt | tail -3 | awk '{print $NF}'`
fi

# filter tracing
grep traceTrackers $traceFiles | sed 's/.*traceTrackers  //'

