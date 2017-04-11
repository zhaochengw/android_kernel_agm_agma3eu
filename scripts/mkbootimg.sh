#!/bin/bash

# common setting
OUTDIR=$1
TARGET_ARCH=$2

# mkbootimg parameter,from device/qcom/<product>/BoardConfig.mk
KERNEL_CMDLINES="console=ttyHSL0,115200,n8 androidboot.console=ttyHSL0 androidboot.hardware=qcom msm_rtb.filter=0x237 ehci-hcd.park=3 androidboot.bootdevice=7824900.sdhci lpm_levels.sleep_disabled=1 earlyprintk"
BASEADDR="--base 0x80000000"
PAGESIZE="--pagesize 2048"
RAMDISK_OFFSET="--ramdisk_offset 0x02000000"
TAGS_OFFSET="--tags_offset 0x01E00000"

# get product name
BOARD_CFG=`cat $OUTDIR/.config | grep CONFIG_HIS_PRODUCT_NAME | sed "s/.*=\"\([A-Za-z0-9_\-]*\)\"/\1/"`

KERNEL=$OUTDIR/arch/$TARGET_ARCH/boot/Image.gz
RAMDISK=./bootimg/ramdisk_${BOARD_CFG}.img
DTIMG=./bootimg/dt.img
BOOTIMG=./bootimg/boot.img

# mkbootimg
if [ -f $KERNEL ]; then
	rm -f $BOOTIMG

	./bootimg/mkbootimg --kernel $KERNEL --ramdisk $RAMDISK --cmdline "$KERNEL_CMDLINES" $BASEADDR $PAGESIZE $RAMDISK_OFFSET $TAGS_OFFSET --dt $DTIMG --output $BOOTIMG
	echo "Build boot.img complete."
else
	echo "Kernel image \"$KERNEL\" not found."
fi

