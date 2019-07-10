#!/usr/bin/env bash

hostname=`hostname`

tmpf="./bcd3fd0d-e1b4-41e5-bca0-26f0f9b64e5a"
echo "
127.0.0.1 localhost
127.0.0.1 $hostname

192.168.5.12 snobook
192.168.5.13 snomiao
192.168.5.13 snotp
192.168.5.20 roslearn
192.168.5.101 car

# The following lines are desirable for IPv6 capable hosts
::1     ip6-localhost ip6-loopback
fe00::0 ip6-localnet
ff00::0 ip6-mcastprefix
ff02::1 ip6-allnodes
ff02::2 ip6-allrouters

13.250.177.223 github.com
151.101.229.194 github.global.ssl.fastly.net
" > $tmpf
sudo cp $tmpf /etc/hosts
rm -f $tmpf
