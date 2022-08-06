#!/usr/bin/env bash
# Monitors the Pico's USB serial port output when it's using pico/stdio_usb.
#
# Usage:
#   monitor_usbserial.sh [sleep_seconds]
#
# Args:
# sleep_seconds: optional time to wait before connecting.
#
set -e -o pipefail

die() {
	echo >&2 "*** ${1}"
	exit 1
}

if [[ ${#} -gt 0 ]]; then
	sleep "${1}"
fi

device=(/dev/serial/by-id/usb-Raspberry_Pi_Pico_*)
stty -F "${device[0]}" 115200
echo "--- Monitoring ${device[0]} ---"
cat "${device[0]}"
