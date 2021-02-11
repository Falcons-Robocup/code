# Copyright 2018 Edwin Schreuder (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python

import serial

serial.Serial() as ser:
    ser.baudrate = 115200
    ser.port = '/dev/ttyS0'
    ser.open()
    ser.write([0x5A, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55])
