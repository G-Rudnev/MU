# #!/usr/bin/env python
# #-*- coding: utf-8 -*-

from pyvesc import VESC
import threading
import cleaner
import time
import math

def ticks(motor, init_val):
    t0 = time.time()
    while time.time() - t0 < T:
        try: 
            # n_ticks = motor.get_measurements().rpm
            print(motor.get_measurements().rpm)
            time.sleep(0.05)
        except:
            print('Bad')
            time.sleep(0.05)
            continue

def smooth_gen(first_val : float, last_val : float, steps : int, delay : float):
    #first_val does not return
    d = (last_val - first_val) / steps
    i = 0
    while 1:
        first_val += d
        yield first_val
        i += 1
        if i == steps:
            break
        time.sleep(delay)

track = 0.53

v0 = 0.2
rpm0 = 60 * v0 / 3.1415926535897932384626433832795 / 0.15

s = 1.0
k = -math.pi

vl = v0 * (1.0 - k * track / 2.0)
rpmL = 60 * vl / 3.1415926535897932384626433832795 / 0.15
vr = v0 * (1.0 + k * track / 2.0)
rpmR = 60 * vr / 3.1415926535897932384626433832795 / 0.15

T = 4

steps = 1
delta_time = 0.05


ignore_clean = True

if __name__ == '__main__':

    time.sleep(1)

    if not ignore_clean:
        myCleaner = cleaner.Cleaner()
        myCleaner.set_cleaner_mode('start')
        time.sleep(4)
    
    with VESC(serial_port = '/dev/right_wheel', start_heartbeat = True, timeout = 0.01) as lmotor:
        with VESC(serial_port = '/dev/left_wheel', start_heartbeat = True, timeout = 0.01) as rmotor:
            # degree0 = lmotor.get_measurements().pid_pos_now   #absolute wheel position in degrees
            
            lmeasurements = None
            while lmeasurements is None:
                lmeasurements = lmotor.get_measurements()
                print('VESC not responding')

            ltachometer0 = lmotor.get_measurements().tachometer
            rtachometer0 = rmotor.get_measurements().tachometer

            for rpm in smooth_gen(0.0, rpm0, steps, delta_time):
                lmotor.set_rpm(int(rpm))
                rmotor.set_rpm(int(rpm))
                print('start rpm:', int(rpm))
            
            # threading.Thread(target = ticks, args = (lmotor, ltachometer0)).start()
            time.sleep(2)

            rrpm_gen = smooth_gen(rpm0, rpmR, steps, delta_time)
            for lrpm in smooth_gen(rpm0, rpmL, steps, delta_time):
                lmotor.set_rpm(int(lrpm))
                rmotor.set_rpm(int(next(rrpm_gen)))
            
            time.sleep(1.0 / v0)

            rrpm_gen = smooth_gen(rpmR, rpm0, steps, delta_time)
            for lrpm in smooth_gen(rpmL, rpm0, steps, delta_time):
                lmotor.set_rpm(int(lrpm))
                rmotor.set_rpm(int(next(rrpm_gen)))
            
            time.sleep(2)

            try:            
                rrpm_gen = smooth_gen(rpm0, 0.0, steps, delta_time)
                for lrpm in smooth_gen(rpm0, 0.0, steps, delta_time):
                    lmotor.set_rpm(int(lrpm))
                    rmotor.set_rpm(int(next(rrpm_gen)))
                    print('stop rpm', lrpm)
                print('L_dist:', (lmotor.get_measurements().tachometer - ltachometer0) / 2000.0 * 3.1415926535897932384626433832795 * 0.15, \
                    'R_dist:', (rmotor.get_measurements().tachometer - rtachometer0) / 2000.0 * 3.1415926535897932384626433832795 * 0.15)
            except:
                lmotor.set_rpm(0)
                rmotor.set_rpm(0)
                print('EXCEPTION!')
                raise

    if not ignore_clean:
        time.sleep(2)
        myCleaner.set_cleaner_mode('stop')

    # time.sleep(1)
    # cleaner.left_brush.set_point(0)
    # print('FINISH')
    # time.sleep(1)
    # exit()
    # cleaner_board.set_cleaner_mode('stop')
    # while(True):
        # print(cleaner_board.get_data())
        # time.sleep(0.05)