#!/usr/bin/env python3
"""Force a USB re-enumeration of the Arduino, equivalent to unplugging it."""
import fcntl, os, re, subprocess, sys

USBDEVFS_RESET = ord('U') << 8 | 20   # _IO('U', 20)
VENDOR = '2341'                        # Arduino SA

def find_device():
    out = subprocess.check_output(['lsusb']).decode()
    for line in out.splitlines():
        if VENDOR in line:
            m = re.match(r'Bus (\d+) Device (\d+)', line)
            if m:
                return f'/dev/bus/usb/{m.group(1)}/{m.group(2)}'
    return None

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else find_device()
    if not path:
        print('Arduino not found on USB bus'); return 1
    print(f'Resetting {path}')
    fd = os.open(path, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)
    print('Reset sent')
    return 0

if __name__ == '__main__':
    sys.exit(main())