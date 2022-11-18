#!/bin/bash


if [ "$#" -lt 2 ]; then
	echo "Please use $0 install.img spi/emmc"
	exit 1
fi

full_image_name=`readlink -f $1`

[ ! -f $full_image_name ] && echo "Please make sure $1 exist!!" && exit 1

mkdir tmp

pushd tools

[ -d tmp ] && rm -rf tmp
[ -d generic ] && rm -rf generic
mkdir tmp

if [ "$2" == "spi" ]; then
	touch tmp/mtdblock0
	chmod 755 tmp/mtdblock0
	touch tmp/mtd0
	chmod 755 tmp/mtd0
	SIZE=16MB
elif [ "$2" == "emmc" ]; then
	touch tmp/mmcblk0
	chmod 755 tmp/mmcblk0
	SIZE=8gb
else
	echo "Not supported flash type"
	echo "Please use $0 install.img spi/emmc"
	exit 1
fi

./install_a_pc $full_image_name $SIZE generic 3
cp tmp/fw_tbl.bin ..
popd

echo "Firmware table image: fw_tbl.bin"
