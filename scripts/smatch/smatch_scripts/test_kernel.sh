#!/bin/bash

NR_CPU=$(cat /proc/cpuinfo | grep ^processor | wc -l)

function usage {
    echo
    echo "Usage:  $0 [smatch options]"
    echo "Compiles the kernel with -j${NR_CPU}"
    echo
    exit 1
}

if [ "$1" = "-h" ] || [ "$1" = "--help" ] ; then
	usage;
fi

SCRIPT_DIR=$(dirname $0)
if [ -e $SCRIPT_DIR/../smatch ] ; then
    CMD=`pwd`/codecheck/smatch

elif which smatch | grep smatch > /dev/null ; then
    CMD=smatch
else
    echo "Smatch binary not found."
    exit 1
fi

OUTDIR=kout
TARGET_ARCH=arm64
# toolchains
PATH=`pwd`/toolchains/aarch64-linux-android-4.9/bin:$PATH
CROSS_COMPILER=aarch64-linux-android-

make clean
find -name \*.c.smatch -exec rm \{\} \;
make O=$OUTDIR ARCH=$TARGET_ARCH CROSS_COMPILE=$CROSS_COMPILER C=1 \
	CHECK="$CMD -p=kernel --file-output $*" -j${NR_CPU}  all
find -name \*.c.smatch -exec cat \{\} \; > warns.txt
find -name \*.c.smatch -exec rm \{\} \;

