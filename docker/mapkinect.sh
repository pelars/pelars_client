#!/bin/bash
IFS=$'\n'
a=''
for i in $( lsusb | grep 045e: ); do
	IFS=' :' read -r -a array <<< "$i"
a="${a} --device /dev/bus/usb/${array[1]}/${array[3]}"
done
echo $a