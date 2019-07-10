#!/bin/bash

# config proxy
echo "proxy configuring"
echo "
export http_proxy="http://"$ip_controller":1080/"
export https_proxy="http://"$ip_controller":1080/"
">>~/.bashrc
