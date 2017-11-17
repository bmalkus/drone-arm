#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

from serial import Serial

dir = os.path.dirname(__file__)

WIDTH = 50

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Usage: {} <probes> <out_file> [<out_file]'.format(__file__))
        sys.exit(1)

    s = Serial('/dev/tty.usbmodem1413', 115200)

    with open('{}/{}'.format(dir, sys.argv[2]), 'wb') as f1, open('{}/{}'.format(dir, sys.argv[3]), 'wb') as f2:
        probes = int(sys.argv[1])
        for i in range(probes):
            done = (i+1)/probes
            print('\r[{0: <{width}}] {percent}% '.format(int(done * WIDTH) * '#', percent=int(done * 100), width=WIDTH), end='')
            line = s.readline()
            f1.write(line[:20])
            f1.write(b'\n')
            f2.write(line[-21:])
    print()
