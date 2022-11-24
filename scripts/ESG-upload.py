#!/usr/bin/env python
"""
This program is used to upload data files to an HP E4400B ESG-D series
vector signal generator connected using a cheap ProLogix clone USB GPIB
adapter.  It assumes the GPIB device ID is 19.

The GPIB adapter uses serial protocol.  It expects to be found at:
`/dev/serial/by-id/usb-10c4_8a5f-if00`.

The device will be in "REMOTE" mode after upload.  You will need to press
the "Local" button to regain user control of the device.

usage: ESG-upload.py <filename>
"""

import serial
import sys
import os
import re

SERIAL_DEVICE = '/dev/serial/by-id/usb-10c4_8a5f-if00'
GPIB_ID = 19

if len(sys.argv) < 2:
    print("usage:", sys.argv[0], "<filename>")
    sys.exit(0)

filename = sys.argv[1]
basename = os.path.basename(filename)
fp = open(filename, 'rb')
data = fp.read()

esg = serial.Serial(SERIAL_DEVICE, 921600) # Chinese ProLogix USB clone.

esg.write(b"++eos 0\n")
esg.write(b"++addr %d\n" % GPIB_ID)
esg.write(b"++auto 1\n")
esg.write(b'*IDN?\n')
print(esg.readline())
esg.write(b"++auto 0\n")

data_len = '%d' % len(data) # Must be done before escaping.

# Escape escape, carriage return, newline and "+" characters (for ProLogix).
data = re.sub(b'\x1b',b'\x1b\x1b', data)
data = re.sub(b'\x0d',b'\x1b\x0d', data)
data = re.sub(b'\x0a',b'\x1b\x0a', data)
data = re.sub(b'\+',b'\x1b+', data)

esg.write(b"++eot_enable\n")
esg.write(b"++eos 3\n")
esg.write(b'MMEM:DATA "%s",#%d%s' % (basename.encode("ASCII"), len(data_len), data_len.encode("ASCII")))
esg.write(data)
esg.write(b"++eos 0\n")
esg.write(b'\n')
