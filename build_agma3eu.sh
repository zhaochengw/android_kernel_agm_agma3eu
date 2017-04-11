#!/bin/bash

# product configure
DEFCONFIG=agma3eu_defconfig

if [ "$1"z = "user"z ]; then
	echo "Build release kernel"
	DEFCONFIG=agma3eu_release_defconfig
fi

OUTDIR=kout
TARGET_ARCH=arm64
PATH=`pwd`/toolchains/aarch64-linux-android-4.9/bin:$PATH
CROSS_COMPILER=aarch64-linux-android-

NR_CPU=$(cat /proc/cpuinfo | grep ^processor | wc -l)

mkdir -p $OUTDIR
make O=$OUTDIR ARCH=$TARGET_ARCH $DEFCONFIG

if [ "$1"z = "check"z ]; then
	echo "run static code analysis"
	shift
	CHECK_TARGET=$*
	. ./scripts/check_code.sh $DEFCONFIG $CHECK_TARGET
else
	rm -rf $OUTDIR/arch/$TARGET_ARCH/boot/dts/*.dtb
	make O=$OUTDIR ARCH=$TARGET_ARCH CROSS_COMPILE=$CROSS_COMPILER KCFLAGS=-mno-android -j${NR_CPU}
	./bootimg/dtbTool -o ./bootimg/dt.img -s 2048 -p $OUTDIR/scripts/dtc/ $OUTDIR/arch/$TARGET_ARCH/boot/dts/

	. ./scripts/mkbootimg.sh $OUTDIR $TARGET_ARCH
fi

