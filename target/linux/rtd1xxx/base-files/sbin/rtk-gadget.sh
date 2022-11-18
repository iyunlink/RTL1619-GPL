#!/bin/sh

if [ $# -ne 1 ]; then
echo "Please use $0 rndis/ecm/msd"
exit 1
fi

if [ "$1" != "rndis" ] && [ "$1" != "ecm" ] && [ "$1" != "msd" ]; then
echo "Not supported mode"
exit 1
fi

GADGET_PATH=/config/usb_gadget/g1

usb_gadget_init()
{
	mount -t configfs none /config

	mkdir $GADGET_PATH

	echo 0x1d6b > $GADGET_PATH/idVendor   # Linux Foundation
	echo 0x0104 > $GADGET_PATH/idProduct  # Multifunction Composite Gadget
	echo 239  > $GADGET_PATH/bDeviceClass # USB_CLASS_MISC
	echo 0x02 > $GADGET_PATH/bDeviceSubClass
	echo 0x01 > $GADGET_PATH/bDeviceProtocol

	mkdir $GADGET_PATH/strings/0x409
	echo 1234567890 > $GADGET_PATH/strings/0x409/serialnumber
	echo "Realtek" > $GADGET_PATH/strings/0x409/manufacturer
	echo "Realtek DHC Gadget" > $GADGET_PATH/strings/0x409/product

	mkdir $GADGET_PATH/configs/c.1
	mkdir $GADGET_PATH/configs/c.1/strings/0x409
	echo "Conf 1" > $GADGET_PATH/configs/c.1/strings/0x409/configuration
	echo 120 > $GADGET_PATH/configs/c.1/MaxPower

	mkdir -p $GADGET_PATH/functions/rndis.usb0
	mkdir -p $GADGET_PATH/functions/mass_storage.usb0
	mkdir $GADGET_PATH/functions/eem.usb0

}


#check if this is a mounted point
[ -d /config ] && mountpoint /config

if [ $? -ne 0 ]; then
	rm -rf /config
	mkdir /config
	usb_gadget_init
else
	udc_string=`cat /config/usb_gadget/g1/UDC`
	[ ! -z $udc_string ] && echo "" > /config/usb_gadget/g1/UDC
	rm $GADGET_PATH/configs/c.1/*.usb0
fi

case "$1" in
	rndis)
		ln -s $GADGET_PATH/functions/rndis.usb0 $GADGET_PATH/configs/c.1
		;;
	ecm)
		ln -s $GADGET_PATH/functions/eem.usb0 $GADGET_PATH/configs/c.1
		;;

	msd)
		[ ! -b /dev/sda1 ] && \
		echo "make sure you have /dev/sda1" && exit 1
		echo /dev/sda1 > $GADGET_PATH/functions/mass_storage.usb0/lun.0/file
		ln -s $GADGET_PATH/functions/mass_storage.usb0 $GADGET_PATH/configs/c.1/
                ;;
esac

echo 0 > /config/usb_gadget/g1/driver_match_existing_only
echo 98050000.dwc3_u3drd > /config/usb_gadget/g1/UDC
