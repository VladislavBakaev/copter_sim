
from math import sqrt, cos, sin, pi
import numpy as np
import threading
import time
from decimal import Decimal
from kalmanFilter import KalmanFilter 

class ImuData():
    def __init__(self):

        self.linearNorthVel = Decimal(0)
        self.linearEastVel = Decimal(0)
        self.linearHeightVel = Decimal(0)

        self.linearNorthAcc = Decimal(0)
        self.linearEastAcc = Decimal(0)
        self.linearHeightAcc = Decimal(0)

        self.northIns = Decimal(0)
        self.heightIns = Decimal(0)
        self.eastIns = Decimal(0)

        self.kalman = KalmanFilter(3)
        self.kalman.weight_k = 0.5

        thread = threading.Thread(target=self.integrateData, args=(Decimal(1/20),))
        thread.start()

    def update_date(self, linear_acc):
        acc = self.kalman.getValue(linear_acc)
        self.linearNorthAcc = Decimal(acc[0])
        self.linearEastAcc = Decimal(acc[1])
        self.linearHeightAcc = Decimal(acc[2])

    def integrateData(self, delay):
        while True:
            northVel = self.linearNorthVel + self.linearNorthAcc*delay
            eastVel = self.linearEastVel + self.linearEastAcc*delay
            heightVel = self.linearHeightVel + self.linearHeightAcc*delay

            self.northIns += self.linearNorthVel*delay + self.linearNorthAcc*delay**2/2
            self.eastIns += self.linearEastVel*delay + self.linearEastAcc*delay**2/2
            self.heightIns += self.linearHeightVel*delay + self.linearHeightAcc*delay**2/2

            self.linearNorthVel = northVel
            self.linearEastVel = eastVel
            self.linearHeightVel = heightVel

            time.sleep(float(delay))

# imu = ImuData()
# imu.update_date([-100,0,0])
# time.sleep(10)
# print(imu.northIns)