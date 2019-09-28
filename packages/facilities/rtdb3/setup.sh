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

# data repo up to date?
srcLib=~/falcons/data/external/libmsgpack-dev-301.tgz
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

# install/check custom-built library: msgpack
srcLib=~/falcons/data/external/libmsgpack-dev-301.tgz
checkFile=/usr/local/lib/libmsgpackc.so.2.0.0
if [ ! -f $checkFile ]; then
    if [ $checkMode = 1 ]; then
        echo "msgpack not installed"
        exit 1
    else
        echo "installing msgpack ..."
        cd $tmpDir
        tar zxf $srcLib
        cd msgpack-c/build/ ; /usr/bin/cmake .. && make && sudo make install
    fi
fi

# install/check custom-built library: zstd
srcLib=~/falcons/data/external/libzstd-dev-135.tgz
checkFile=/usr/local/lib/libzstd.so.1.3.5
if [ ! -f $checkFile ]; then
    if [ $checkMode = 1 ]; then
        echo "zstd not installed"
        exit 1
    else
        echo "installing zstd ..."
        cd $tmpDir
        tar zxf $srcLib
        cd zstd
        make && sudo make install
    fi
fi

# for lmdb, we require python installation next to standard apt-get install
checkDir=/usr/local/lib/python2.7/dist-packages/lmdb
if [ ! -d $checkDir ]; then
    if [ $checkMode = 1 ]; then
        echo "lmdb for python not installed"
        exit 1
    else
        echo "installing lmdb for python ..."
        sudo pip install lmdb
    fi
fi

# cleanup
if [ -d $tmpDir ]; then
    rm -rf $tmpDir
fi
if [ $checkMode = 0 ]; then
    echo "everything should be OK now, please verify with:"
    echo "   rosmake -s rtdb2"
fi

