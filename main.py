# NOTE: Most of this is made redundant by the MPU's DMP, this is mostly just implemented for the sake of learning
import math
from mpu6050 import mpu6050
from time import time

# in m/s^2
GRAVITY = 9.8

# Bias for complimentary filter, lower number = more reliance on accelerometer
ALPHA = 0.3

imu = mpu6050(0x68)
startTime = time()

while True:
    timeElapsed = time() - startTime
    startTime = time()

    accelData = imu.get_accel_data()
    gyroData = imu.get_gyro_data()

    # Calculate roll and pitch from accelerometer
    phiHat = math.atan(accelData['y']/accelData['z'])
    thetaHat = math.asin(accelData['x']/GRAVITY)

    # Euler rate transform
    phiDot = gyroData['x'] + math.sin(phiHat) * math.tan(thetaHat) * gyroData['y'] * math.tan(thetaHat) * gyroData['z']
    thetaDot = math.cos(phiHat) * gyroData['y'] - math.sin(phiHat) * gyroData['z']

    # Update filter

    phiHat = (1 - ALPHA) * (phiHat + timeElapsed * phiDot) + phiHat * ALPHA
    thetaHat = (1 - ALPHA) * (thetaHat + timeElapsed * thetaDot) + thetaHat * ALPHA

    print(f"Phi Hat: {phiHat}")
    print(f"Theta Hat: {thetaHat}")





