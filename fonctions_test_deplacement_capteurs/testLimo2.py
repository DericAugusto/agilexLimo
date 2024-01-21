#!/usr/bin/env python3
# coding=UTF-8
from pylimo import limo
import time

limo = limo.LIMO()
limo.EnableCommand()

print("\n\nCode running...\n\n")

while True:
    data1 = limo.GetLinearVelocity()
    data2 = limo.GetAngularVelocity()
    data3 = limo.GetSteeringAngle()
    data4 = limo.GetLateralVelocity()
    data5 = limo.GetControlMode()
    #data6 = limo.GetLeftWheelOdom() # ça ne marche pas
    data7 = limo.GetBatteryVoltage()
    data8 = limo.GetErrorCode()
    data9 = limo.GetIMUAccelData()
    data10 = limo.GetIMUGyroData()
    #data11 = limo.GetIMUYaw() # ça ne marche pas
    #data12 = limo.GetIMUPitch() # ça ne marche pas
    #data13 = limo.GetIMURoll() # ça ne marche pas

    print("linear velocity: " + str(data1) + "\n" + "type: " + str(type(data1)) + "\n------------------------\n")
    print("angular velocity: " + str(data2) + "\n" + "type: " + str(type(data2)) + "\n------------------------\n")
    print("steering angle: " + str(data3) + "\n" + "type: " + str(type(data3)) + "\n------------------------\n")  
    print("lateral velocity: " + str(data4) + "\n" + "type: " + str(type(data4)) + "\n------------------------\n")
    print("control mode: " + str(data5) + "\n" + "type: " + str(type(data5)) + "\n------------------------\n")
    #print("left wheel odometer: " + data6 + "\n" + "type: " + type(data6) + "\n------------------------\n")
    print("battery voltage: " + str(data7) + "\n" + "type: " + str(type(data7)) + "\n------------------------\n")
    print("error code: " + str(data8) + "\n" + "type: " + str(type(data8)) + "\n------------------------\n")
    print("data from accelerometer: " + str(data9) + "\n" + "type: " + str(type(data9)) + "\n------------------------\n")
    print("gyroscope data: " + str(data10) + "\n" + "type: " + str(type(data10)) + "\n------------------------\n")
    #print("yaw from IMU : " + data11 + "\n" + "type: " + type(data11) + "\n------------------------\n")
    #print("pitch from IMU: " + data12 + "\n" + "type: " + type(data12) + "\n------------------------\n")
    #print("roll from IMU: " + data13 + "\n" + "type: " + type(data13) + "\n------------------------\n")
    
    time.sleep(0.9)
    
