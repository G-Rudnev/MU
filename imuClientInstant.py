#!/usr/bin/env python

import sys
import socket
from mpu6050 import mpu6050
from ahrs.filters import Madgwick
from ahrs.filters import Mahony
from ahrs import Quaternion

import math
import numpy as np
import time
import threading

def Pack_Params(from_params : dict) -> str:
    s = ""
    for key in from_params:
        s += (key + ":=" + from_params[key] +  ";;")
    return str(len(s)) + "\n" + s + "\n"

globalVals = {
    "x" : "0.0",
    "y1" : "0.0",
    "y2" : "0.0",
    "xlim_min" : "-0.001",
    "xlim_max" : "1.0",
    "ylim_min" : "-1.0",
    "ylim_max" : "1.0",
    "reset" : "1"
}

flea = go()

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.settimeout(2.0)
HOST = '192.168.43.71'
PORT = 4004

client.connect((HOST, PORT))

sensor = mpu6050(0x68)
sensor.open()

sensor.set_filter_range(mpu6050.FILTER_BW_5)

Trig = [True]

N = 10
plots_N = 0

xAccel = yAccel = yawAccel = pitchAccel = rollAccel = 0.0
pitchSpeed = rollSpeed = 0.0
pitch = roll = 0.0

xAccels = np.zeros([N])
yAccels = np.zeros([N])

xOffset = yOffset = 0.0
xAccel_prev = yAccel_prev = 0.0

t0 = time.time()

n = 0

q = np.array([1.0, 0.0, 0.0, 0.0])
# filter = Madgwick(q0 = q.copy(), frequency = 100.0)
filter = Mahony(q0 = q.copy(), frequency = 100.0)

accelData_prev = sensor.get_accel_data()
accelData_prevTime = time.time()

gyroData_prev = sensor.get_gyro_data()
gyroData_prevTime = time.time()

time.sleep(0.01)

while True:

    if (Trig[0]):

        Trig[0] = False

        globalVals["ylim_min"] = "-1"
        globalVals["ylim_max"] = "1"

        # xAccel = yAccel = yawAccel = pitchAccel = rollAccel = 0.0
        xSpeed = ySpeed = yawSpeed = 0.0
        x = y = yaw = 0.0

    accelData = sensor.get_accel_data()
    accelData_deltaTime = time.time() - accelData_prevTime
    accelData_prevTime += accelData_deltaTime

    gyroData = sensor.get_gyro_data()
    gyroData_deltaTime = time.time() - gyroData_prevTime
    gyroData_prevTime += gyroData_deltaTime


    #Body Frame в https://ahrs.readthedocs.io/en/latest/nomenclature.html,
    #правая тройка векторов
    q = filter.updateIMU(q, gyr = np.array([-gyroData['y'], gyroData['x'], gyroData['z']]) * 0.01745329252, \
                           acc = np.array([accelData['y'], -accelData['x'], accelData['z']]))
    angles = Quaternion(q).to_angles() * 57.29578 * 2.4

    #знаки и оси - как сориентируешь IMU

    yawAccel = (gyroData['z'] - gyroData_prev['z']) / gyroData_deltaTime
    yawSpeed += yawAccel * gyroData_deltaTime
    yaw += (yawSpeed - yawAccel * gyroData_deltaTime / 2.0) * gyroData_deltaTime
    

    pitchAccel = (gyroData['y'] - gyroData_prev['y']) / gyroData_deltaTime
    pitchSpeed += pitchAccel * gyroData_deltaTime
    pitch += (pitchSpeed - pitchAccel * gyroData_deltaTime / 2.0) * gyroData_deltaTime


    rollAccel = (gyroData['x'] - gyroData_prev['x']) / gyroData_deltaTime
    rollSpeed += rollAccel * gyroData_deltaTime
    roll += (rollSpeed - rollAccel * gyroData_deltaTime / 2.0) * gyroData_deltaTime

    if n < N:
        M = n
    else:
        M = N - 1
    n += 1
    for i in range(M, 0, -1):
        xAccels[i] = xAccels[i - 1]
        yAccels[i] = yAccels[i - 1]

    xAccels[0] = accelData['x'] + 9.80665 * math.sin(pitch * 0.01745329252)
    yAccels[0] = accelData['y'] - 9.80665 * math.sin(roll * 0.01745329252)
    
    if (np.var(xAccels[:(M + 1)]) < 0.1):    #offset moving estimation
        xOffset = 0.0157 * (xAccels[0] + xAccel_prev) + 0.969 * xOffset
        xAccel_prev = xAccels[0]

    xAccel = xAccels[0] - xOffset
    xSpeed += xAccel * accelData_deltaTime
    x += (xSpeed - xAccel * accelData_deltaTime / 2.0) * accelData_deltaTime

    if (np.var(yAccels[:(M + 1)]) < 0.1 or n <= N):
        yOffset = 0.0157 * (yAccels[0] + yAccel_prev) + 0.969 * yOffset
        yAccel_prev = yAccels[0]

    yAccel = yAccels[0] - yOffset
    ySpeed += yAccel * accelData_deltaTime
    y += (ySpeed - yAccel * accelData_deltaTime / 2.0) * accelData_deltaTime

    accelData_prev = accelData
    gyroData_prev = gyroData

    if (not (plots_N % 5)):

        T = gyroData_prevTime - t0
        val1 = yaw
        val2 = angles[2]

        if (not (plots_N % 600)):
            globalVals["reset"] = "1"
            globalVals["xlim_min"] = str(T)
        else:
            globalVals["reset"] = "0"

        globalVals["x"] = str(T)
        globalVals["y1"] = str(val1)
        globalVals["y2"] = str(val2)

        if val1 <= val2:
            val = val1
        else:
            val = val2
        if (val < float(globalVals["ylim_min"])):
            globalVals["ylim_min"] = str(val - 0.3 * abs(val))

        if val1 > val2:
            val = val1
        else:
            val = val2
        if (val > float(globalVals["ylim_max"])):
            globalVals["ylim_max"] = str(val + 0.3 * abs(val))
        
        # globalVals["xlim_min"] = str(0)
        globalVals["xlim_max"] = str(T + 0.5)

        client.send(bytes(Pack_Params(globalVals), 'utf-8'))


        #использовать, если события начинают опережать передачу данных
        # while True:

        #     echo = client.recv(256).decode('utf-8')
        #     if echo == globalVals["x"]:
        #         break

    plots_N += 1
    time.sleep(0.01)