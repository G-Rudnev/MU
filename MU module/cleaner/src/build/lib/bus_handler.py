#!/usr/bin/env python
# -*- coding: utf-8 -*-

__version__ = "0.0.1"

import serial

import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu


class Bus():


    def __init__(self, port, baudrate = 115200, debug = False, timeout = 1.0, port_forward = False):

        self.bus = modbus_rtu.RtuMaster(serial.Serial(port=port, baudrate=baudrate, bytesize=8, parity='N', stopbits=1, xonxoff=0, rtscts=port_forward, dsrdtr=port_forward, exclusive = True)) 

        self.bus.set_verbose(debug)

        self.bus.set_timeout(timeout)




