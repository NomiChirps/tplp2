#!/usr/bin/env bash

# FIXME: just make monitor_usbserial.sh do this itself
while true;  do
    $PWD/tools/monitor_usbserial.sh
    sleep 0.2s
done