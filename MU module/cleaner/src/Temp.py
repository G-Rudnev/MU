#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pyvesc import VESC
import cleaner
import time

if __name__ == '__main__':

    cleaner_board = cleaner.Cleaner()
    # print('CLEAN:', cleaner_board._read_data()["FRESH_WATER_LEVEL"])
    # print('DIRTY:', cleaner_board._read_data()["DIRTY_WATER_LEVEL"])

    cleaner_board.set_cleaner_mode('stop')

    time.sleep(20)

    # cleaner_board.set_cleaner_mode('stop')/

    # cleaner_board.left_brush.set_point(30)
    # cleaner_board.water_pump_control(255)
    # time.sleep(10)
    # cleaner_board.water_pump_control(0)


    # while True:
    #     cleaner_board.poll_joy()
    #     # data = cleaner_board.get_data()
    #     # print(cleaner_board.get_data()["US_SENSOR_1"] /10.0, \
    #     # cleaner_board.get_data()["US_SENSOR_2"]/10.0, \
    #     # cleaner_board.get_data()["US_SENSOR_3"]/10.0, \
    #     # cleaner_board.get_data()["US_SENSOR_4"]/10.0, \
    #     # cleaner_board.get_data()["IR_SENSOR_1"], \
    #     # cleaner_board.get_data()["IR_SENSOR_2"])
    #     # print(cleaner_board.get_data()["INPUT_VOLTAGE"])
    #     # cleaner_board.right_brush.set_point(30)
    #     # cleaner_board.set_cleaner_mode('start')
    #     # cleaner_board.vacuum_cleaner_control(False)
    #     # cleaner_board.front_actuator_control('up')
    #     time.sleep(0.02)