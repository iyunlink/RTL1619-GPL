#!/bin/bash

# only check usb drd wifi device now

if [ "$(echo $INTERFACE | cut -c 1-4)" = "wlan" -a "$(ls -l /sys/class/net | grep $INTERFACE | grep drd)" ]; then
  RADIO_NO=$(echo $INTERFACE | cut -c 5)
  if [ "$ACTION" = "add" ]; then
    wifi up radio$RADIO_NO
  elif [ "$ACTION" = "remove" ]; then
    wifi down radio$RADIO_NO
  fi
fi
