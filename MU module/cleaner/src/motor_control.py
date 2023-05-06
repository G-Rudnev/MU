from pyvesc import VESC
import threading
from threading import Lock, Thread
import time

class VESC_motor(object):

    def __init__(self, port_name, motor_type, limit = 0.3):

        self.type = motor_type
        self.motor = VESC(serial_port = port_name)
        self.limit = limit

        self._motor_thread = threading.Thread(target=self._get_motor_data, args=())
        self._motor_thread.daemon = True
        self._motor_thread.start()

        self.motor_data = None
        self.motor_mutex = Lock()


    def _get_motor_data(self):
        while True:
            try:
                time.sleep(0.05)
                self.motor_mutex.acquire()           
                self.motor_data = self.motor.get_measurements()
                self.motor_mutex.release()
            except Exception as e:
                print(e)
                print("error on reading motor")
                self.motor.stop_heartbeat()
                self.motor_mutex.release()
                return False    


    def get_data(self):

        """Чтение данных с датчиков расстояния

        Args:
            None
        Returns:
            * Словарь с ключами: 
                | "MOTOR_CURRENT" - потребляемый ток мотора (А), 
                | "RPM" - скорость вращения вала (только для колес) (об/мин), 
                | "INPUT_VOLTAGE" - входное напряжение (V)
                | "TICKS" - количество тиков энкодеров 
                | "TICKS_ABS" - абсолютное количество тиков энкодера
                | "FAULT_CODE" - код ошибки (см. описание ошибок)
                | "DUTY" - текущая подаваемая скважность ШИМ на мотор

        """        

        if self.motor_data:
            data = {}
            #data["MOTOR_CURRENT"] = self.motor_data.current_motor
            data["RPM"] = self.motor_data.rpm
            data["INPUT_VOLTAGE"] = self.motor_data.v_in
            data["TICKS"] = self.motor_data.tachometer
            data["TICKS_ABS"] = self.motor_data.tachometer_abs
            data["FAULT_CODE"] = self.motor_data.mc_fault_code
            data["DUTY"] = self.motor_data.duty_cycle_now
            return data


    def set_point(self, val):
        try:
            self.motor_mutex.acquire()  
            if self.type == "wheel":  
                if val > self.limit:
                    val = self.limit
                if val < self.limit * -1:
                    val = self.limit * -1         
                ret =  self.motor.set_duty_cycle(val)
            if self.type == "brush": 
                ret =  self.motor.set_current(val)   
            self.motor.stop_heartbeat()
            self.motor_mutex.release()
        except Exception as e:
            print(e)
            print("error")
            self.motor.stop_heartbeat()
            self.motor_mutex.release()                
            return False   
        return ret

