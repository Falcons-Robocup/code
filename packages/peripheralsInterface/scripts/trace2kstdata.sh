#!/usr/bin/env bash
#
# JFEI 2016-12-11
# Convert tracing to a file which can be loaded in KST.




# default outputfile
outputfile=/var/tmp/kstdata.txt

# guess input
arg=$1
if [ -z "$arg" ]; then
    folder=`newest_logdir.py`
    tracefile=`ls -1 $folder/trace_*_halMw*.txt | tail -1`
else
    if [ -d "$arg" ]; then
        folder=$arg
        tracefile=`ls -1 $folder/trace_*_halMw*.txt | tail -1`
    else
        tracefile=$arg
        if [ ! -f "$arg" ]; then
            echo "ERROR: don't know what to do with given argument"
            exit 1
        fi
    fi
fi
if [ ! -f "$tracefile" ]; then
    echo "ERROR: something went wrong"
    exit 1
fi
echo "input file: $tracefile"

# clear outputfile
if [ -f "$outputfile" ]; then
    rm $outputfile
fi

# write header, Changes have impact, also change in main_motors.cpp
echo "mLeSetpoint;mLeVelocity;mLeError;mLeProportional;mLeIntegral;mLeDerivative;mLePIDOutput;mLeDisplDist;mLeDisplTicks;mLePWMm;RiSetpoint;mRiVelocity;mRiError;mRiProportional;mRiIntegral;mRiDerivative;mRiPIDOutput;mRiDisplDist;mRiDisplTicks;mRiPWM;mReSetpoint;mReVelocity;mReError;mReProportional;mReIntegral;mReDerivative;mRePIDOutput;mReDisplDist;mReDisplTicks;mRePWM;" >> $outputfile
echo "m/s;m/s;m/s;m/s;m/s;m/s;m/s;m;Ticks;%;m/s;m/s;m/s;m/s;m/s;m/s;m/s;m;Ticks;%;m/s;m/s;m/s;m/s;m/s;m/s;m/s;m;Ticks;%;">> $outputfile


# extract lines
grep kstdata $tracefile | sed 's/.*kstdata//' >> $outputfile

echo "file written: $outputfile"

