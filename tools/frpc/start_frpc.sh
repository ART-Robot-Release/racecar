#!/bin/bash
#
# configure frpc
# Snowstar (snomiao@gmail.com)
# v2019.06.30
# 


SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)
sudo cp "$SHELL_FOLDER/frpc" /usr/bin/frpc
sudo mkdir /etc/frp
sudo cp "$SHELL_FOLDER/frpc.ini" /etc/frp/frpc.ini
sudo cp "$SHELL_FOLDER/systemd/frpc.service" /etc/systemd/system/
sudo chmod 777 /usr/bin/frpc


sudo systemctl daemon-reload
sudo systemctl enable frpc.service
sudo systemctl start frpc.service

# chmod 777 "$SHELL_FOLDER/frpc"
# "$SHELL_FOLDER/frpc" -c "$SHELL_FOLDER/frpc.ini"
