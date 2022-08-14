#!/usr/bin/env bash
# Monitors the Pico's USB serial port output when it's using pico/stdio_usb.

echo "Continuously monitoring for USB-serial connection. CTRL+C to stop."

while true; do
	device=(/dev/serial/by-id/usb-Raspberry_Pi_Pico_*)
	if [[ ! -e ${device[0]} ]]; then
		sleep 0.1
		continue
	fi
	stty -F "${device[0]}" 115200
	echo "--- Opening ${device[0]} ---"
	cat "${device[0]}"
	echo "--- Lost connection to ${device[0]} ---"
	sleep 1
done
