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

if [ -e `pwd`/scripts/smatch/smatch ] ; then
	CMD=`pwd`/scripts/smatch/smatch
else
	echo "======== CAN NOT FIND smatch ========"
fi

find -name \*.c.smatch -exec rm \{\} \;
make O=$OUTDIR ARCH=$TARGET_ARCH CROSS_COMPILE=$CROSS_COMPILER C=1 \
	CHECK="$CMD -p=kernel --file-output" -j${NR_CPU}  all
find -name \*.c.smatch -exec cat \{\} \; > warns.txt
find -name \*.c.smatch -exec rm \{\} \;

