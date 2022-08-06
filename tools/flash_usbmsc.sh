#!/usr/bin/env bash
# Mounts the Pico's USB mass storage device and copies a file into it.
# The Pico must already be in bootloader mode for this script to work.
# First argument is the file to copy.
# TODO: see if we can force it into bootloader mode automatically from here.

set -e -o pipefail

die() {
	echo >&2 "*** ${1}"
	exit 1
}

if [[ ${#} != 1 ]]; then
	die "Usage: $0 firmware.uf2"
fi

if [[ ! -e ${1} ]]; then
	die "${1} does not exist" >&2
fi

label="RPI-RP2"
mountpoint="/run/media/${USER}/${label}"

if [[ ! -e ${mountpoint} ]]; then
	udisksctl mount -b "/dev/disk/by-label/${label}"
fi

cp "${1}" "${mountpoint}"/
echo
# Wait a sec for the device to reboot and establish a USB connection for whatever we're doing next.
sleep 1
echo "Success."
