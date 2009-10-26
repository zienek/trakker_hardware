#!/bin/sh

major=-1
module="at91adc"

/sbin/rmmod $module
if [ "$?" != "0" ]
then
echo ...error ignored
fi

rm /dev/at91adc0

/sbin/insmod ./$module.ko
if [ "$?" = "0" ]
then
rm -rf /dev/at91adc0
major=`cat /proc/devices | awk "\\$2==\"at91adc\" {print \\$1}"`
if [ "$?" = "0" ]
then
mknod /dev/at91adc0 c $major 0
echo "Created nodes /dev/at91adc0"
fi
fi
