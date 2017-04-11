#! /bin/bash

if [ $# -lt 2 ]; then
	echo "Error:"
	echo "  Please input product name and arch"
else
	# $1: product name  $2: arm or arm64
	perl ./release_script/mk_kernel_release.pl $1 $2
	rm -rf ./release_script
fi

