#!/bin/bash
# Script to be run on the RPi at startup

#https://ardupilot.org/mavproxy/docs/getting_started/starting.html#streamrate 
# run mavproxy in a screen session so we can access it at any time
# by running `screen -r mavproxy` (to get out do CTRL+a then CTRL+d)
mavproxy_cmd='/home/pi/.local/bin/mavproxy.py
    --baudrate=115200
    --streamrate=0
    --source-system=1
    --source-component=192
    --master="/dev/serial0"
    --out="127.0.0.1:14550"
    --out="127.0.0.1:14551"
    --aircraft="DRover"
    --state-basedir=/home/pi
    --default-modules="link"
    '
mavproxy_cmd=$(echo $mavproxy_cmd|tr -d '\n')

echo "${mavproxy_cmd}"
screen -X -S mavproxy kill # kill mavproxy if already open
echo "Starting mavproxy"
screen -dmS mavproxy bash -c 'eval $mavproxy_cmd ; bash'

# start drover
echo "Starting DRover main"
drover_main.py

echo "dead"