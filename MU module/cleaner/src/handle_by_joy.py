#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cleaner
import time

import sys
sys.path.append('/home/pi/Desktop/Robo/pyRobo')

from Lidar import*

if __name__ == '__main__':

    time.sleep(10.0)

    try:
        lid0 = Lidar.Create('02000201000700050000000601040504')

        lid0.Start()

        time.sleep(25.0)

        lid0.Stop()

    except:
        pass

    cleaner_board = cleaner.Cleaner()

    while True:
        cleaner_board.poll_joy()
        time.sleep(0.02)