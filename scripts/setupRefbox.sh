#!/bin/bash
#
# Setup the refbox application.
# 


set -e

PATH_PROCESSING=~/processing-3.5.3
PATH_REFBOX=${FALCONS_DATA_PATH}/external/refbox2019beta/mslrb2015

# require data repo to be uptodate
if [ ! -d $PATH_REFBOX ]; then
    echo "ERROR: data repo not uptodate"
    exit 1
fi

# install processing if not done already
if [ ! -d $PATH_PROCESSING ]; then
    cd ~
    echo
    echo "downloading processing-3.5.3 (140MB) ..."
    # alternatively we could also store it in data repo... ? (seems a bit big...)
    echo
    wget http://download.processing.org/processing-3.5.3-linux64.tgz
    echo
    echo "unpacking ..."
    echo
    tar zxf processing-3.5.3-linux64.tgz
    echo
    echo "processing-3.5.3 has been installed"
    echo
fi

