#!/bin/bash
# Attach / detach the Videomancer (USB 16d0:14db) to WSL via usbipd-win.
# While attached, Windows loses the device (its USB drive and MIDI too).
#   ./usb_attach.sh          attach to WSL   (-> /dev/ttyACM0)
#   ./usb_attach.sh detach   give it back to Windows
USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"
cd /mnt/c || exit 1
if [ "${1:-}" = detach ]; then
    "$USBIPD" detach --hardware-id 16d0:14db
else
    "$USBIPD" attach --wsl --hardware-id 16d0:14db
fi
