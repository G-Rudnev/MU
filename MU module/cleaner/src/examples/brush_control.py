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
        
        cleaner_board.left_brush.set_point(15)
        cleaner_board.right_brush.set_point(-15)

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
        right_brush_data = cleaner_board.right_brush.get_data()
        left_brush_data = cleaner_board.left_brush.get_data()

        print(left_brush_data["DUTY"], left_brush_data["DUTY"])

        time.sleep(0.1)





