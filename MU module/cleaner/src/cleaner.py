#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#
"""Модуль для работы с контроллером поломойщика по шине Modbus RTU."""

__status__ = "Developing"
__version__ = "0.0.1"

import bus_handler
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

import motor_control
import joy_control

from threading import Lock, Thread, Event
import yaml
import time

from functools import wraps

#Board registers
_REG_INPUT_VOLTAGE     = 5
_REG_SENSORS_STATE     = 6
_REG_HEARTBEAT		   = 7

_REG_FRONT_ACTUATOR    = 10
_REG_REAR_ACTUATOR     = 11
_REG_WATER_PUMP        = 14
_REG_VACUUM_CLEAN      = 16

_REG_FRESH_FULL        = 20
_REG_FRESH_EMPTY       = 21

_REG_DIRTY_FULL        = 22
_REG_DIRTY_EMPTY       = 23

_REG_FLOW_METER        = 24

_REG_LED_1             = 30
_REG_LED_2             = 31
_REG_LED_3             = 32
_REG_LED_4             = 33

_REG_US_SENSOR_1       = 40
_REG_US_SENSOR_2       = 41
_REG_US_SENSOR_3       = 42
_REG_US_SENSOR_4       = 43
_REG_US_SENSOR_5       = 44

_REG_IR_SENSOR_1       = 45
_REG_IR_SENSOR_2       = 46

_REG_DISTANCE_THRESH   = 47


import signal
import sys


class Cleaner(object):
    """Класс для работы с мойщиком.

    """
    def __init__(self):

        self.logger = modbus_tk.utils.create_logger("console")

        with open('/home/pi/Desktop/Robo/cleaner/src/config.yaml', 'r') as stream:
            self.params = yaml.safe_load(stream)
            print(self.params['bus']['port_name']) 

        self.addr = self.params['bus']['main_board_id'] 
        self.master = bus_handler.Bus(port = self.params['bus']['port_name'], baudrate = self.params['bus']['baudrate'], debug = self.params['bus']['debug'], timeout = self.params['bus']['timeout'])                 

        time.sleep(1)

        self.mutex = Lock()

        # self.set_distance_thresh(0, self.params['sensors']['us_sens_1_thresh'])
        # self.set_distance_thresh(1, self.params['sensors']['us_sens_2_thresh'])
        # self.set_distance_thresh(2, self.params['sensors']['us_sens_3_thresh'])
        # self.set_distance_thresh(3, self.params['sensors']['us_sens_4_thresh'])

        self.cleaning_status = 0

        self.brush_cur_down = self.params['adaptive_brush_level']['brush_cur_down']
        self.brush_cur_up   = self.params['adaptive_brush_level']['brush_cur_up']
        self.brush_cur_good_min = self.params['adaptive_brush_level']['brush_cur_good_min']
        self.brush_cur_good_max = self.params['adaptive_brush_level']['brush_cur_good_max']
        self.brush_cur_setp     = self.params['adaptive_brush_level']['brush_cur_setp']              

        if self.params['brushes']['use_brushes'] == True:
            self.left_brush = motor_control.VESC_motor(self.params['brushes']['left_brush_port'], 'brush')
            self.right_brush = motor_control.VESC_motor(self.params['brushes']['right_brush_port'], 'brush')

        if self.params['wheels']['use_wheels'] == True:
            self.left_wheel = motor_control.VESC_motor(self.params['wheels']['left_wheel_port'], 'wheel', self.params['wheels']['duty_limit'])
            self.right_wheel = motor_control.VESC_motor(self.params['wheels']['right_wheel_port'], 'wheel', self.params['wheels']['duty_limit'])

        signal.signal(signal.SIGINT, self.signal_handler)

        self.device_data = {}
        self.vacuum_cleaner_state = False
        self.moving_status = True       

        self.poll_thread = Thread(target=self._poll_board, daemon = True)
        self.poll_thread.start() 

        time.sleep(2)
        self.ready = Event()
        self.ready.set()

        if self.params['gamepad']['use_gamepad'] == True:
            self.joy = joy_control.XboxController(self.params['gamepad']['type'])                       

    #######PRIVATE FUNCTIONS#######

    def signal_handler(self,sig, frame):
        print('Exit')
        self.ready.clear()
        if self.params['wheels']['use_wheels'] == True:
            self.right_wheel.motor.stop_heartbeat()
            self.left_wheel.motor.stop_heartbeat()
        if self.params['brushes']['use_brushes'] == True:    
            self.right_brush.motor.stop_heartbeat()
            self.left_brush.motor.stop_heartbeat()
        sys.exit(0)


    def _getSignedNumber(self,number, bitLength):
        mask = (2 ** bitLength) - 1
        if number & (1 << (bitLength - 1)):
            return number | ~mask
        else:
            return number & mask   


    def except_decorator(fn):
        @wraps(fn)
        def wrapped(self,*args):
            try:
                self.ready.set()
                return fn(self,*args)
            except Exception as e:
                self.logger.error("%s", e)
                self.ready.clear()
                self.mutex.release()
                return False
        return wrapped


    def _poll_board(self):
        while True:
            try:
                self.device_data = self._read_data()
                self.set_heartbeat()
                if self.cleaning_status == 1:
                    self._adaptive_level_poll()
                    if self.params['brushes']['use_brushes'] == True:
                        self.left_brush.set_point(self.brush_cur_setp * -1)
                        self.right_brush.set_point(self.brush_cur_setp)                        
                time.sleep(0.05)
            except Exception as e:
                self.ready.clear()
                print(e)
                print("error while reading device")


    @except_decorator
    def _read_data(self):
        data = {}

        self.mutex.acquire()
        values = self.master.bus.execute(1, cst.READ_HOLDING_REGISTERS, 0, 50)
        self.mutex.release()
        
        data["FLOW_METER"] = values[_REG_FLOW_METER]

        data["INPUT_VOLTAGE"] = values[_REG_INPUT_VOLTAGE]

        data["FRONT_ACTUATOR_CMD"] = self._getSignedNumber(values[_REG_FRONT_ACTUATOR], 16)
        data["REAR_ACTUATOR_CMD"] = self._getSignedNumber(values[_REG_REAR_ACTUATOR], 16)

        data["WATER_PUMP_CMD"] = values[_REG_WATER_PUMP]
        data["VACUUM_CLEAN_CMD"] = values[_REG_VACUUM_CLEAN]

        data["US_SENSOR_1"] = values[_REG_US_SENSOR_1]
        data["US_SENSOR_2"] = values[_REG_US_SENSOR_2]
        data["US_SENSOR_3"] = values[_REG_US_SENSOR_3]
        data["US_SENSOR_4"] = values[_REG_US_SENSOR_4]
        data["US_SENSOR_5"] = values[_REG_US_SENSOR_5]
        data["IR_SENSOR_1"] = values[_REG_IR_SENSOR_1]
        data["IR_SENSOR_2"] = values[_REG_IR_SENSOR_2]
        # data["SENSORS_STATE"] = values[_REG_SENSORS_STATE]

        if values[_REG_FRESH_FULL]:
            data["FRESH_WATER_LEVEL"] = 'FULL'
        elif values[_REG_FRESH_EMPTY]:
            data["FRESH_WATER_LEVEL"] = 'OK'
        elif values[_REG_FRESH_EMPTY] == 0 and values[_REG_FRESH_FULL] == 0:
            data["FRESH_WATER_LEVEL"] = 'EMPTY'             


        if values[_REG_DIRTY_FULL]:
            data["DIRTY_WATER_LEVEL"] = 'FULL'
        elif values[_REG_DIRTY_EMPTY]:
            data["DIRTY_WATER_LEVEL"] = 'OK'
        elif values[_REG_DIRTY_EMPTY] == 0 and values[_REG_DIRTY_FULL] == 0:
            data["DIRTY_WATER_LEVEL"] = 'EMPTY'     

        return data


    def _adaptive_level_poll(self):
        if self.params['brushes']['use_brushes'] == True and self.params['adaptive_brush_level']['use_adaptive_control'] == True:

            right_brush_current = self.right_brush.get_data()
            left_brush_current  = self.left_brush.get_data()

            if right_brush_current < self.brush_cur_down and left_brush_current < self.brush_cur_down:
                #self.pub_front_wheel.publish(-250)
                self.front_actuator_control("down")
                #print("ACT_DOWN")
            elif right_brush_current > self.brush_cur_up and left_brush_current > self.brush_cur_up:
                #self.pub_front_wheel.publish(250)
                self.front_actuator_control("up")
                #print("ACT_UP")
            elif right_brush_current > self.brush_cur_good_min and right_brush_current < self.brush_cur_good_max and left_brush_current > self.brush_cur_good_min and left_brush_current < self.brush_cur_good_max:           
                #self.pub_front_wheel.publish(250)
                self.front_actuator_control("up")
                #print("ACT_NORM")


    @except_decorator
    def set_heartbeat(self):      
        self.mutex.acquire()
        ret =  self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_HEARTBEAT, output_value=1)           
        self.mutex.release()    
        return ret  


    def poll_joy(self):
        """Опрос геймпада и управление исполнительными механизмами по командам с него

        Args:
            None
        Returns:
            None

        """          
        try:
            self.joy.poll_joy()
            val = self.joy.read()
            #print(val)
            if val[0][8]:
                if val[0][3] > 0.9:
                    self.front_actuator_control('up')
                if val[0][3] < -0.9:
                    self.front_actuator_control('down')
                if val[0][3] > -0.9 and val[0][3] < 0.9:
                    self.front_actuator_control('stop')    

            if val[0][9]:
                if val[0][3] > 0.9:
                    self.rear_actuator_control('up')
                if val[0][3] < -0.9:
                    self.rear_actuator_control('down')
                if val[0][3] > -0.9 and val[0][3] < 0.9:
                    self.rear_actuator_control('stop')

            if val[1][3]:
                 self.vacuum_cleaner_control(not(self.vacuum_cleaner_state))

            if val[1][2]:
                    self.stop_all()                      

            if val[1][8]:
                if self.cleaning_status == 1:
                    self.set_cleaner_mode("stop")
                else:
                    self.set_cleaner_mode("start")                                                                                      

            if val[0][4]: 
                out_val = self.joy.joystickToDiff(val[0][0],val[0][1],-1.0,1.0,-1.0,1.0)
                if self.params['wheels']['use_wheels'] == True:
                    self.left_wheel.set_point(out_val[0])
                    self.right_wheel.set_point(out_val[1])
                if self.cleaning_status == 1:
                    if val[0][1] > 0.05:
                        self.water_pump_control(int(self.joy.map(val[0][1], 0.05, 1.0, self.params['water_pump']['min_power'], self.params['water_pump']['max_power'])))
                    else:
                        self.water_pump_control(0)              
        except Exception as e:
            print(e)
            print("error while polling gamepad")


    ############################ 


    def stop_moving(self,state):
        self.moving_status = state          


    def get_data(self):
        """Чтение данных с платы контроллера мойщика

        Args:
            None
        Returns:
            * Словарь с ключами: 
                | "FLOW_METER", 
                | "INPUT_VOLTAGE", 
                | "FRONT_ACTUATOR_CMD" 
                | "REAR_ACTUATOR_CMD" 
                | "WATER_PUMP_CMD" 
                | "VACUUM_CLEAN_CMD" 
                | "US_SENSOR_1" 
                | "US_SENSOR_2" 
                | "US_SENSOR_3" 
                | "US_SENSOR_4" 
                | "IR_SENSOR_1"
                | "IR_SENSOR_2"
                | "FRESH_WATER_LEVEL" 
                | "DIRTY_WATER_LEVEL" 

        """        
        return self.device_data


    # @except_decorator
    def get_statuses(self):
        """Получение статусов мойки, и работы исполнительных механизмов
        
        Args:
            None
        Returns:
            * Словарь с ключами: 
                | "CLEANING_STATUS", 
                | "FRONT_ACTUATOR_CMD", 
                | "REAR_ACTUATOR_CMD" 
                | "WATER_PUMP_CMD" 
                | "VACUUM_CLEAN_CMD"
                | "MOVING_STATUS" 
                | "SENSORS_STATE"  

        """ 
        return self.cleaning_status


    @except_decorator
    def front_actuator_control(self, val):
        """Управление передним актуатором

        Args:
            val(str): 
                | "up" - поднятие актуатора
                | "down" - опускание актуатора
                | "stop" - оcтанов актуатора              
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка

        """         
        self.mutex.acquire()
        # print(val)
        if val == 'up':
            ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_FRONT_ACTUATOR, output_value=255)
        elif val == 'down':   
            ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_FRONT_ACTUATOR, output_value=-255)
        elif val == 'stop':   
            ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_FRONT_ACTUATOR, output_value=0)              
        self.mutex.release()    
        return ret    


    @except_decorator
    def rear_actuator_control(self, val):
        """Управление задним актуатором

        Args:
            val(str): 
                | "up" - поднятие актуатора
                | "down" - опускание актуатора 
                | "stop" - оcтанов актуатора              
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка

        """         
        self.mutex.acquire()
        if val == 'up':
            ret =  self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_REAR_ACTUATOR, output_value=255)
        elif val == 'down': 
            ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_REAR_ACTUATOR, output_value=-255)
        elif val == 'stop': 
            ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_REAR_ACTUATOR, output_value=0)                
        self.mutex.release()    
        return ret     


    @except_decorator    
    def water_pump_control(self, val):
        """Управление насососм чистой воды.

        Args:
            val(int): мощность работы насоса (0-255). 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка

        """
        # print(int(val))         
        self.mutex.acquire()
        ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_WATER_PUMP, output_value=val)
        self.mutex.release()
        return ret


    @except_decorator
    def vacuum_cleaner_control(self, val):
        """Управление пылесосом.

        Args:
            val(Bool): Режим работы. True - вкл, False - выкл. 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка

        """
        self.vacuum_cleaner_state = val        
        self.mutex.acquire()
        if val:
            ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_VACUUM_CLEAN, output_value=255)
        else:
            ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_VACUUM_CLEAN, output_value=0)    
        self.mutex.release()
        return ret


    @except_decorator
    def LED_control(self, channel, state):
        """Управление подсветкой.

        Args:
            channel(int): Канал подсветки (0-1) 
            state(bool): Режим работы. True - вкл, False - выкл
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка

        """         
        self.mutex.acquire()
        ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_LED_1 + channel, output_value=state)
        self.mutex.release()
        return ret


    def stop_all(self):
        """Выключение всех исполнительных механизмов

        Args:
            None
        Returns:
            None 

        """          
        self.cleaning_status = 0
        self.vacuum_cleaner_control(False)
        self.water_pump_control(0)
        self.rear_actuator_control("up")
        self.front_actuator_control("up")
        if self.params['wheels']['use_wheels'] == True:
            self.right_wheel.set_point(0)
            self.left_wheel.set_point(0)
        if self.params['brushes']['use_brushes'] == True:    
            self.right_brush.set_point(0)
            self.left_brush.set_point(0)


    @except_decorator
    def set_cleaner_mode(self, mode, water_pump : int = 127):
        """Управление правым колесом.

        Args:
            mode(str) Режим мойки:
                | 'start' - начало мойки
                | 'stop' - остановка мойки 
        Returns:
            * True если отправка команды прошла успешно
            * False если при отправке команды произошла ошибка

        """
        # print(mode)          
        if mode == 'start':
            self.cleaning_status = 1
            self.rear_actuator_control('down')
            self.front_actuator_control('down')
            time.sleep(0.1)
            self.water_pump_control(water_pump)
            time.sleep(0.1)
            if self.params['brushes']['use_brushes']:
                self.left_brush.set_point(self.brush_cur_setp * -1)
                time.sleep(0.2)
                self.right_brush.set_point(self.brush_cur_setp)
            time.sleep(0.1)
            self.vacuum_cleaner_control(True)

        if mode == 'stop':
            self.cleaning_status = 0
            self.vacuum_cleaner_control(False)
            self.rear_actuator_control('up')
            self.front_actuator_control('up')
            self.water_pump_control(0)
            if self.params['brushes']['use_brushes']:
                self.left_brush.set_point(0)
                self.right_brush.set_point(0)            

        return True    
    
    '''
        # def get_distances(self):
        #     """Чтение данных с датчиков расстояния

        #     Args:
        #         None
        #     Returns:
        #         * Словарь с ключами: 
        #             | "US_SENSOR_1", 
        #             | "US_SENSOR_2", 
        #             | "US_SENSOR_3" 
        #             | "US_SENSOR_4" 
        #             | "US_SENSOR_5"
        #             | "IR_SENSOR_1"
        #             | "IR_SENSOR_2" 

        #     """          
        #     data = {}
        #     data["US_SENSOR_1"] = self.device_data["US_SENSOR_1"]
        #     data["US_SENSOR_2"] = self.device_data["US_SENSOR_2"]
        #     data["US_SENSOR_3"] = self.device_data["US_SENSOR_3"]
        #     data["US_SENSOR_4"] = self.device_data["US_SENSOR_4"]
        #     data["IR_SENSOR_1"] = self.device_data["IR_SENSOR_1"]
        #     data["IR_SENSOR_2"] = self.device_data["IR_SENSOR_2"]
        #     return data


        # @except_decorator
        # def set_distance_thresh(self, sensor, val):
        #     """
        #     DOES NOTHING!!! (04.05.2023)
        #     Установка порогого значения для датчика расстояния.

        #     Args:
        #         sensor(int): Номер датчика (0-4) 
        #         val(int): Значение в мм (0-2500)
        #     Returns:
        #         * True если отправка команды прошла успешно
        #         * False если при отправке команды произошла ошибка

        #     """         
        #     self.mutex.acquire()
        #     ret = self.master.bus.execute(self.addr, cst.WRITE_SINGLE_REGISTER, _REG_DISTANCE_THRESH + sensor, output_value=val)
        #     self.mutex.release()
        #     return ret
    '''


if __name__ == '__main__':

    cleaner = Cleaner()
    while True:
        cleaner.poll_joy()
        time.sleep(0.02)            