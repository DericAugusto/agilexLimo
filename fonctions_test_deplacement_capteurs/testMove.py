#!/usr/bin/env python3
# coding=UTF-8

from pylimo import limo
import time

limo = limo.LIMO()
limo.EnableCommand()

print("\n\nCode running...\n\n")

while True:
  linear_vel = 0.1
  angular_vel = -0.01
  print("\n\nSetting:\nlinear_vel: " + str(linear_vel) + "\nangular_vel: "+ str(angular_vel))
  limo.SetMotionCommand(linear_vel=linear_vel,angular_vel=angular_vel)
  time.sleep(10)
  
  linear_vel = 0
  angular_vel = 0
  lateral_velocity = 0
  steering_angle = 0
  print("\n\nNow setting:\nlinear_vel: " + str(linear_vel) + "\nangular_vel: "+ str(angular_vel) + "\nlateral_velocity: "+ str(lateral_velocity) + "\nsteering_angle: "+ str(steering_angle))
  limo.SetMotionCommand(linear_vel=linear_vel, angular_vel=angular_vel, lateral_velocity=lateral_velocity, steering_angle=steering_angle)
  time.sleep(3)
  
  linear_vel = 0.5
  angular_vel = -0.08
  lateral_velocity = 0
  steering_angle = 0
  print("\n\nNow setting:\nlinear_vel: " + str(linear_vel) + "\nangular_vel: "+ str(angular_vel) + "\nlateral_velocity: "+ str(lateral_velocity) + "\nsteering_angle: "+ str(steering_angle))
  limo.SetMotionCommand(linear_vel=linear_vel, angular_vel=angular_vel, lateral_velocity=lateral_velocity, steering_angle=steering_angle)
  time.sleep(10)
  
  linear_vel = 0
  angular_vel = 0
  lateral_velocity = 0
  steering_angle = 0
  print("\n\nNow setting:\nlinear_vel: " + str(linear_vel) + "\nangular_vel: "+ str(angular_vel) + "\nlateral_velocity: "+ str(lateral_velocity) + "\nsteering_angle: "+ str(steering_angle))
  limo.SetMotionCommand(linear_vel=linear_vel, angular_vel=angular_vel, lateral_velocity=lateral_velocity, steering_angle=steering_angle)
  time.sleep(3)
  
  linear_vel = 0.5
  angular_vel = 0
  lateral_velocity = 0.5
  steering_angle = 0
  print("\n\nNow setting:\nlinear_vel: " + str(linear_vel) + "\nangular_vel: "+ str(angular_vel) + "\nlateral_velocity: "+ str(lateral_velocity) + "\nsteering_angle: "+ str(steering_angle))
  limo.SetMotionCommand(linear_vel=linear_vel, angular_vel=angular_vel, lateral_velocity=lateral_velocity, steering_angle=steering_angle)
  time.sleep(10)
  
  linear_vel = 0
  angular_vel = 0
  lateral_velocity = 0
  steering_angle = 0
  print("\n\nNow setting:\nlinear_vel: " + str(linear_vel) + "\nangular_vel: "+ str(angular_vel) + "\nlateral_velocity: "+ str(lateral_velocity) + "\nsteering_angle: "+ str(steering_angle))
  limo.SetMotionCommand(linear_vel=linear_vel, angular_vel=angular_vel, lateral_velocity=lateral_velocity, steering_angle=steering_angle)
  time.sleep(3)
  
  linear_vel = 0.5
  angular_vel = 0
  lateral_velocity = 0.0
  steering_angle = 0.5
  print("\n\nNow setting:\nlinear_vel: " + str(linear_vel) + "\nangular_vel: "+ str(angular_vel) + "\nlateral_velocity: "+ str(lateral_velocity) + "\nsteering_angle: "+ str(steering_angle))
  limo.SetMotionCommand(linear_vel=linear_vel, angular_vel=angular_vel, lateral_velocity=lateral_velocity, steering_angle=steering_angle)
  time.sleep(10)
  
  linear_vel = 0
  angular_vel = 0
  lateral_velocity = 0
  steering_angle = 0
  print("\n\nNow setting:\nlinear_vel: " + str(linear_vel) + "\nangular_vel: "+ str(angular_vel) + "\nlateral_velocity: "+ str(lateral_velocity) + "\nsteering_angle: "+ str(steering_angle))
  limo.SetMotionCommand(linear_vel=linear_vel, angular_vel=angular_vel, lateral_velocity=lateral_velocity, steering_angle=steering_angle)
  time.sleep(3)