#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cleaner
import time

if __name__ == '__main__':

    cleaner_board = cleaner.Cleaner()

    while True:
        cleaner_board.poll_joy()
        time.sleep(0.02)