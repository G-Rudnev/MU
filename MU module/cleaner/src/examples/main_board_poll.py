#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cleaner
import time

if __name__ == '__main__':

    cleaner_board = cleaner.Cleaner()

    while True:
        values = cleaner_board.get_data()
        print(values)
        time.sleep(0.1)