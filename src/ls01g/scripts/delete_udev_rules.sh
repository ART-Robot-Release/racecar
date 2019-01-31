#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to laser"
echo "sudo rm   /etc/udev/rules.d/laser.rules"
sudo rm   /etc/udev/rules.d/laser.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
