#!/usr/bin/env bash
set -e

device=/dev/serial/by-id/usb-15ba_Olimex_OpenOCD_JTAG_ARM-USB-OCD-H_OL56E93E-if01-port0
stty -F "${device}" 115200
echo "Monitoring UART/RS232 output. Ctrl+C to stop."
cat "${device}"
