#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cleaner
import time

if __name__ == '__main__':

    cleaner_board = cleaner.Cleaner()
    start = time.time()
    elapsed = 0
    while elapsed < 10:
        elapsed = time.time() - start
        
        cleaner_board.left_wheel.set_point(0.05)
        cleaner_board.right_wheel.set_point(-0.05)

        '''
        get_data() возвращает словарь со следующими ключами:
                "MOTOR_CURRENT" - потребляемый ток мотора (А), 
                "RPM" - скорость вращения вала (только для колес) (об/мин), 
                "INPUT_VOLTAGE" - входное напряжение (V)
                "TICKS" - количество тиков энкодеров 
                "TICKS_ABS" - абсолютное количество тиков энкодера
                "FAULT_CODE" - код ошибки
                "DUTY" - текущая подаваемая скважность ШИМ на мотор
        '''        
        right_wheel_data = cleaner_board.right_wheel.get_data()
        left_wheel_data = cleaner_board.left_wheel.get_data()

        print(left_wheel_data["TICKS"], left_wheel_data["TICKS"])

        time.sleep(0.1)





