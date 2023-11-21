#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import time
limo=limo.LIMO()
limo.EnableCommand()
while True:
    time.sleep(0.1)
    rapport = f'linear velocity: {limo.GetLinearVelocity():d} \n\
               angular velocity: {limo.GetAngularVelocity():d} \n\
               steering angle: {limo.GetSteeringAngle():d} \n\
               lateral velocity: {limo.GetLateralVelocity():d}\n\
               control mode: {limo.GetControlMode():d}\n\
               left wheel odometer: {limo.GetLeftWheelOdom():d}\n\
               battery voltage: {limo.GetBatteryVoltage():d}\n\
               error code: {limo.GetErrorCode():d}\n\
               data from accelerometer: {limo.GetIMUAccelData():d}\n\
               gyroscope data: {limo.GetIMUGyroData():d}\n\
               yaw from IMU : {limo.GetIMUYaw():d}\n\
               pitch from IMU: {limo.GetIMUPitch():d}\n\
               roll from IMU: {limo.GetIMURoll():d}'

   print(rapport)