#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  laser"
echo "ls01g usb cp210x connection as /dev/laser , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy laser.rules to  /etc/udev/rules.d/"
echo "`rospack find ls01g`/scripts/laser.rules"
sudo cp `rospack find ls01g`/scripts/laser.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
