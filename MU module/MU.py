import sys
import time
import threading
import numpy as np
from numpy.linalg import norm as norm 
from math import inf as inf
from math import pi as pi
from math import pow as pow
from math import tan as tan
from math import sin as sin
from math import cos as cos
from math import atan as atan
from math import atan2 as atan2
from RoboMath import*   #always needed
from Config import*     #always needed

from mpu6050 import mpu6050

__all__ = ['IMU', 'Peripherals']

class IMU:
    """
        SINGLE IMU OBJECT (non static, for instances use Create function).\n
        Each instance executes in new thread (when Start() being call)
    """

    _rndKey = hash("IMU")    #it is just a random number
    _NumOfIMUs = 0
    
    def __init__(self, rndKey, imuID):

        if (rndKey != IMU._rndKey):
            print("Use IMU.Create() function only!!!", exc = True)

        #IDs
        self.imuID = imuID
        self.imuAddress = int(mainDevicesPars[f"imuAddress_ID{self.imuID}"])
        IMU._NumOfIMUs += 1

        self._frameID = 0   #all captured frames
        self.frameID = 0    #the last public (uploaded) frame

        self.sensor = mpu6050(self.imuAddress)

        #MAIN PARAMETERS
        self.mount = np.zeros([2])    #local coordinates
        for i, el in enumerate(mainDevicesPars[f"imuMount_ID{self.imuID}"].split()):
            if i > 1:
                break
            self.mount[i] = float(el)
        self.mountPhi = float(mainDevicesPars[f"imuMountPhi_ID{self.imuID}"])
        self.sampleTime = float(mainDevicesPars[f"imuSampleTime_ID{self.imuID}"])
        self.accelBufferDeep = 12   #buffer of accelData for moving dispersion observation
        self.g = self.sensor.GRAVITIY_MS2

        #PRIVATE
        self._xSpeed = self._ySpeed = self._yawSpeed = self._pitchSpeed = self._rollSpeed = 0.0
        self._x = self._y = self._yaw = self._pitch = self._roll = 0.0

        #PUBLIC
        self.s = 0.0
        self.alpha = 0.0
        self.alphaSpeed = 0.0
        self.timeLabel = 0.0
        
        #THREADS AND LOCKS
        self._thread = []
        self._mutex = threading.RLock()

        #EVENTS AND HANDLES
        self.ready = threading.Event()
        self.ready.clear()
        self._isFree = threading.Event()
        self._isFree.set()

    def _Start_wrp(self, f):
        def func(*args, **kwargs):
            if (self._isFree.wait(timeout = 0.5)):  #for the case of delayed stopping
                self._thread = threading.Thread(target = f, args = args, kwargs = kwargs)
                self._thread.start()
                while self._thread.isAlive() and not self.ready.is_set():
                    time.sleep(0.01)
                # if self.ready.is_set():
                #     time.sleep(0.5) #do something if you need
            return self.ready.is_set()
        return func

    def Start(self, onlyAlpha = True):
        self._isFree.clear()
        try:

            #SETTING UP CONNECTION
            self.sensor.open()
            self.sensor.set_filter_range(mpu6050.FILTER_BW_5)

            xAccel = yAccel = yawAccel = pitchAccel = rollAccel = 0.0
            self._xSpeed = self._ySpeed = self._yawSpeed = self._pitchSpeed = self._rollSpeed = 0.0
            self._x = self._y = self._yaw = self._pitch = self._roll = 0.0

            xAccels = np.zeros([self.accelBufferDeep])
            yAccels = np.zeros([self.accelBufferDeep])

            xAccel_prev = yAccel_prev = 0.0

            xOffset = yOffset = 0.0

            n = 0

            gyroData_prev = self.sensor.get_gyro_data()
            gyroData_prevTime = time.time()

            accelData_prev = self.sensor.get_accel_data()
            accelData_prevTime = time.time()

            time.sleep(self.sampleTime)

            self.ready.set()
            while (self.ready.is_set() and threading.main_thread().is_alive()):

                #GET DATA
                gyroData = self.sensor.get_gyro_data()
                gyroData_deltaTime = time.time() - gyroData_prevTime
                gyroData_prevTime += gyroData_deltaTime

                if (not onlyAlpha):
                    
                    accelData = self.sensor.get_accel_data()
                    accelData_deltaTime = time.time() - accelData_prevTime
                    accelData_prevTime += accelData_deltaTime

                    #ACCEL DATA BUFFER LOOP
                    if n < self.accelBufferDeep:
                        M = n
                    else:
                        M = self.accelBufferDeep - 1
                    n += 1
                    for i in range(M, 0, -1):
                        xAccels[i] = xAccels[i - 1]
                        yAccels[i] = yAccels[i - 1]

                with self._mutex:

                    #CALCULATE GYRO DATA
                    yawAccel = (gyroData['z'] - gyroData_prev['z']) / gyroData_deltaTime
                    self._yawSpeed += yawAccel * gyroData_deltaTime
                    self._yaw += (self._yawSpeed - yawAccel * gyroData_deltaTime / 2.0) * gyroData_deltaTime

                    self._frameID += 1

                    if (not onlyAlpha):

                        pitchAccel = (gyroData['y'] - gyroData_prev['y']) / gyroData_deltaTime
                        self._pitchSpeed += pitchAccel * gyroData_deltaTime
                        self._pitch += (self._pitchSpeed - pitchAccel * gyroData_deltaTime / 2.0) * gyroData_deltaTime

                        rollAccel = (gyroData['x'] - gyroData_prev['x']) / gyroData_deltaTime
                        self._rollSpeed += rollAccel * gyroData_deltaTime
                        self._roll += (self._rollSpeed - rollAccel * gyroData_deltaTime / 2.0) * gyroData_deltaTime


                        #IMPROVE ACCELS WITH ROLL AND PITCH
                        xAccels[0] = accelData['x'] + self.g * sin(self._pitch * 0.01745329252)
                        yAccels[0] = accelData['y'] - self.g * sin(self._roll * 0.01745329252)


                        #CALCULATE ACCEL DATA
                        if (np.var(xAccels[:(M + 1)]) < 0.1):    #offset decimated estimation by moving variance
                            xOffset = 0.0157 * (xAccels[0] + xAccel_prev) + 0.969 * xOffset  #LPF, estimating current fofset
                            xAccel_prev = xAccels[0]

                        xAccel = xAccels[0] - xOffset
                        self._xSpeed += xAccel * accelData_deltaTime
                        self._x += (self._xSpeed - xAccel * accelData_deltaTime / 2.0) * accelData_deltaTime

                        if (np.var(yAccels[:(M + 1)]) < 0.1):
                            yOffset = 0.0157 * (yAccels[0] + yAccel_prev) + 0.969 * yOffset
                            yAccel_prev = yAccels[0]

                        yAccel = yAccels[0] - yOffset
                        self._ySpeed += yAccel * accelData_deltaTime
                        self._y += (self._ySpeed - yAccel * accelData_deltaTime / 2.0) * accelData_deltaTime
                    
                    #FINISH
                    # accelData_prev = accelData    #useless
                    gyroData_prev = gyroData

                time.sleep(self.sampleTime)
            
        except:
            if (sys.exc_info()[0] is not RoboException):
                print('IMU ' + str(self.imuID) + ' error! ' + str(sys.exc_info()[1]) + '; line: ' + str(sys.exc_info()[2].tb_lineno), log = True)
        finally:
            self.ready.clear()
            #CLOSE CONNECTION
            if self.sensor.isOpen.is_set():
                self.sensor.close()
            self._isFree.set()

    def Stop(self):
    ####WE HAVE TO USE NON-BLOCKING STOP BECAUSE OF SOME WORK LOGIC MOMENTS, TO BE RELEVANT IN START THERE IS A _isFree.wait() ON START
        self.ready.clear()

    @classmethod
    def Create(cls, imuID):
        try:
            imu = IMU(cls._rndKey, imuID)
            imu.Start = imu._Start_wrp(imu.Start)
            return imu
        except:
            if (sys.exc_info()[0] is not RoboException):
                print(f"IMU {imu.imuID} error! {sys.exc_info()[1]}; line: {sys.exc_info()[2].tb_lineno}")
            return None

    def Get_alphaSpeed(self, pauseIfOld = 0.002, default_alphaSpeed = 0.0):
        """
            No resetting assumed!
            Alpha speed in radian / s and is not divided by 2.
        """
        with self._mutex:    #let there will be no exceptions next   
            if not self.ready.is_set():
                self.alphaSpeed = default_alphaSpeed
                return -1
            if self.frameID != self._frameID:
                self.frameID = self._frameID
                self.alphaSpeed = self._yawSpeed * 0.01745329252
                return 1
        time.sleep(pauseIfOld)
        return 0
    
    def Get_alpha(self, speed = None, speedFusion = 1.0, default_alpha = 0.0, reset = True):
        """
            Alpha is actually measured alpha / 2 - a little feature of GoRobo.
            Alpha speed in radian / s and is not divided by 2.
        """

        with self._mutex:    #let there will be no exceptions next    
                
            if speed is not None:
                self._yawSpeed = speedFusion * speed * 57.295779513 + (1.0 - speedFusion) * self._yawSpeed
                self._frameID += 1

            self.timeLabel = time.time()

            if self.ready.is_set():
                self.alpha = self._yaw * 0.00872664626 #degrees to radian and divided by 2
                if reset:
                    self._yaw = 0.0
                return 0
            else:
                self.alpha = default_alpha
                self._yaw = 0.0
                return -1

    def Get_s_alpha(self, speed_s_alpha = (None, None), speedsFusion = (1.0, 1.0), default_s_alpha = (0.0, 0.0), reset = True):
        """
            Alpha is actually measured alpha / 2 - a little feature of GoRobo.
            Alpha speed in radian / sec and is not divided by 2.
            speed_s_alpha: s is a speed along arc.
            Use s only if sensor is in the center of mass!
        """

        with self._mutex:    #let there will be no exceptions next    
            
            if speed_s_alpha[0] is not None:
                self._xSpeed = speedsFusion[0] * speed_s_alpha[0] + (1.0 - speedsFusion[0]) * self._xSpeed
                self._pitchSpeed = 0.0 #there is a cross axis influency
                self._pitch = 0.0

            if speed_s_alpha[1] is not None:
                self._yawSpeed = speedsFusion[1] * speed_s_alpha[1] * 57.295779513 + (1.0 - speedsFusion[1]) * self._yawSpeed
                self._frameID += 1

            self.timeLabel = time.time()

            if self.ready.is_set():
                self.s, self.alpha = self._x, self._yaw * 0.00872664626 #degrees to radian and divided by 2
                if reset:
                    self._x = self._yaw = 0.0
                return 0
            else:
                self.s, self.alpha = default_s_alpha
                self._x = self._yaw = 0.0
                return -1
            
class Peripherals:
    """
        SINGLE IMU OBJECT (non static, for instances use Create function).\n
        Each instance executes in new thread (when Start() being call)
    """

    OK = 0
    US_1 = 1
    US_2 = 2
    US_3 = 4
    US_4 = 8
    US_5 = 16
    IR_1 = 32
    IR_2 = 64
    NO_WATER = 128
    FULL_WATER = 256
    LOW_VOLTAGE = 512

    OBSTACLES = US_1 + US_2 + US_3 + US_4 + US_5
    FLOOR = IR_1 + IR_2
    WATER = NO_WATER + FULL_WATER
    FREEZERS = OBSTACLES + FLOOR
    CLEANSTOPPERS = WATER + LOW_VOLTAGE
    ALL = OBSTACLES + FLOOR + WATER + LOW_VOLTAGE

    LITERALS = {
        'EMPTY' : 0,
        'OK' : 1,
        'FULL' : 2
    }

    _rndKey = hash("Peripherals")    #it is just a random number
    
    def __init__(self, rndKey, carrier):

        if (rndKey != Peripherals._rndKey):
            print("Use Peripherals.Create() function only!!!", exc = True)

        self.carrier = carrier

        self._frameID = 0   #all captured frames
        self.frameID = 0    #the last public (uploaded) frame

        #MAIN PARAMETERS
        self.us_thres = int(globalPars["us_thres"])
        self.voltage_thres = float(globalPars["voltage_thres"])
        self.pause = 0.025

        #PRIVATE
        self._data = {}

        #PUBLIC
        self.info : int = Peripherals.OK
        self.voltage = 0.0
        self.fresh_water_level = 0
        self.dirty_water_level = 0

        #THREADS AND LOCKS
        self._thread = []
        self._mutex = threading.RLock()

        #EVENTS AND HANDLES
        self.ready = threading.Event()
        self.ready.clear()
        self._isFree = threading.Event()
        self._isFree.set()

    def _Start_wrp(self, f):
        def func(*args, **kwargs):
            if (self._isFree.wait(timeout = 0.5)):  #for the case of delayed stopping
                self._thread = threading.Thread(target = f, args = args, kwargs = kwargs)
                self._thread.start()
                while self._thread.isAlive() and not self.ready.is_set():
                    time.sleep(0.01)
                # if self.ready.is_set():
                #     time.sleep(0.5) #do something if you need
            return self.ready.is_set()
        return func

    def Start(self):

        self._isFree.clear()
        try:
            
            #SETTING UP CONNECTION
            if (not self.carrier.ready.is_set()):
                raise Exception("Cannot start peripherals")

            self.ready.set()
            while (self.ready.is_set() and threading.main_thread().is_alive()):

                #GET DATA
                if (not self.carrier.ready.is_set()):
                    raise Exception("Peripherals fault")

                data = self.carrier.get_data()
                
                with self._mutex:
                    if (data is not self._data):
                        self._data = data
                        self._frameID += 1

                time.sleep(self.pause)
            
        except:
            if (sys.exc_info()[0] is not RoboException):
                print('Peripherals error! ' + str(sys.exc_info()[1]) + '; line: ' + str(sys.exc_info()[2].tb_lineno), log = True)
        finally:
            self.ready.clear()
            #CLOSE CONNECTION

            self._isFree.set()

    def Stop(self):
    ####WE HAVE TO USE NON-BLOCKING STOP BECAUSE OF SOME WORK LOGIC MOMENTS, TO BE RELEVANT IN START THERE IS A _isFree.wait() ON START
        self.ready.clear()

    @classmethod
    def Create(cls, carrier):
        try:
            peri = cls(cls._rndKey, carrier)
            peri.Start = peri._Start_wrp(peri.Start)
            return peri
        except:
            if (sys.exc_info()[0] is not RoboException):
                print(f"Peripherals error! {sys.exc_info()[1]}; line: {sys.exc_info()[2].tb_lineno}")
            return None

    def Get(self, pauseIfOld = 0.0):
        with self._mutex:    #let there will be no exceptions next   

            if not self.ready.is_set():
                return -1
            
            if self.frameID != self._frameID:
                self.frameID = self._frameID
                data = self._data
                
                self.voltage = data["INPUT_VOLTAGE"]
                self.fresh_water_level = Peripherals.LITERALS[data["FRESH_WATER_LEVEL"]]
                self.dirty_water_level = Peripherals.LITERALS[data["DIRTY_WATER_LEVEL"]]
                
                self.info = Peripherals.OK
                if (self.voltage < self.voltage_thres):
                    self.info |= Peripherals.LOW_VOLTAGE
                if (self.fresh_water_level == Peripherals.LITERALS['EMPTY']):
                    self.info |= Peripherals.NO_WATER
                if (self.dirty_water_level == Peripherals.LITERALS['FULL']):
                    self.info |= Peripherals.FULL_WATER
                if (data["US_SENSOR_1"] < self.us_thres):
                    self.info |= Peripherals.US_1
                if (data["US_SENSOR_2"] < self.us_thres):
                    self.info |= Peripherals.US_2
                if (data["US_SENSOR_3"] < self.us_thres):
                    self.info |= Peripherals.US_3
                if (data["US_SENSOR_4"] < self.us_thres):
                    self.info |= Peripherals.US_4
                if (data["US_SENSOR_5"] < self.us_thres):
                    self.info |= Peripherals.US_5
                if (data["IR_SENSOR_1"] == 1):
                    self.info |= Peripherals.IR_1
                if (data["IR_SENSOR_2"] == 1):
                    self.info |= Peripherals.IR_2
                return 1
            
        time.sleep(pauseIfOld)
        return 0

    def MakeInfoMessage(self):
        msg = ""
        if (self.info & Peripherals.US_1):
            msg += "Obstacle by US 1, "
        if (self.info & Peripherals.US_2):
            msg += "Obstacle by US 2, "
        if (self.info & Peripherals.US_3):
            msg += "Obstacle by US 3, "
        if (self.info & Peripherals.US_4):
            msg += "Obstacle by US 4, "
        if (self.info & Peripherals.US_5):
            msg += "Obstacle by US 5, "
        if (self.info & Peripherals.IR_1):
            msg += "No floor surface by IR 1, "
        if (self.info & Peripherals.IR_2):
            msg += "No floor surface by IR 2, "
        if (self.info & Peripherals.LOW_VOLTAGE):
            msg += "Low input voltage, "
        if (self.info & Peripherals.NO_WATER):
            msg += "Fresh water tank is empty, "
        if (self.info & Peripherals.FULL_WATER):
            msg += "Dirty water tank is full, "
        if len(msg) > 0:
            return msg[:-2]
        else:
            return "Peripherals is OK"

class Peripherals_Flea(Peripherals):

    def Start(self):

        self._isFree.clear()
        try:

            with self._mutex:
                self.info = Peripherals.OK
                self.voltage = 27.0
                self.fresh_water_level = Peripherals.LITERALS['FULL']
                self.dirty_water_level = Peripherals.LITERALS['EMPTY']
                self.pause = 0.1

            self.ready.set()

            while (self.ready.is_set() and threading.main_thread().is_alive()):

                #GET DATA

                time.sleep(self.pause)
            
        except:
            if (sys.exc_info()[0] is not RoboException):
                print('Peripherals error! ' + str(sys.exc_info()[1]) + '; line: ' + str(sys.exc_info()[2].tb_lineno), log = True)
        finally:
            self.ready.clear()
            #CLOSE CONNECTION

            self._isFree.set()

    def Get(self, pauseIfOld = 0.0):

        with self._mutex:    #let there will be no exceptions next   

            if not self.ready.is_set():
                return -1
            
            time.sleep(pauseIfOld)
            return 1
        
    def MakeInfoMessage(self):
        return "Peripherals is OK"