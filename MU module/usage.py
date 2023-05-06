#Вырвано из контекста
from Config import*

from MU import IMU
from MU import Peripherals as Peripherals

import cleaner

#IMU
imu = IMU.Create(0)
imu.Stop() #check if creation successeful, unhandled exception otherwise

cleaner = cleaner.Cleaner()

#PERIPHERALS
peri = Peripherals.Create(cleaner)
peri.Stop() #check if creation successeful, unhandled exception otherwise

#......