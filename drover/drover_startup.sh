#!/bin/bash
# Script to be run on the RPi at startup

#https://ardupilot.org/mavproxy/docs/getting_started/starting.html#streamrate 
# run mavproxy in a screen session so we can access it at any time
# by running `screen -r mavproxy` (to get out do CTRL+a then CTRL+d)
mavproxy_cmd='mavproxy.py
    --baudrate=115200
    --streamrate=0
    --source-system=1
    --source-component=192
    --master="/dev/serial0"
    --out="127.0.0.1:14550"
    --out="127.0.0.1:14551"
    --aircraft="DRover"
    --state-basedir=/var/log
    '
    
screen -X -S mavproxy kill # kill mavproxy if already open
screen -dmS mavproxy bash -c '${mavproxy_cmd} ; bash'
echo "Started mavproxy"

# TODO start drover
bash