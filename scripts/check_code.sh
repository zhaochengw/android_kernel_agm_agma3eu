#!/bin/bash

DEFCONFIG=$1

shift
CHECK_TARGET=$*
if [ "$1" = "--output-file" ]; then
	REAL_TARGET=$2
else
	REAL_TARGET=$1
fi

if [ -f $REAL_TARGET ]; then
	touch $REAL_TARGET
elif [ -d $REAL_TARGET ]; then
	rm -rf $OUTDIR/$REAL_TARGET
elif [ $REAL_TARGET = "all" ]; then
	rm -rf $OUTDIR && mkdir -p $OUTDIR
else
	echo "Please input check target!"
fi

if [ "$REAL_TARGET" = "all" ]; then
	make O=$OUTDIR ARCH=$TARGET_ARCH $DEFCONFIG
	. scripts/smatch/smatch_scripts/check_all_kernel.sh
else
	. scripts/smatch/smatch_scripts/kchecker $CHECK_TARGET
fi

