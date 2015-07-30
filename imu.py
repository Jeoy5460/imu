#!/usr/bin/python
# -*- coding: utf-8 -*-
Kp = 2.0                 # proportional gain governs rate of convergence to accelerometer/magnetometer
Ki = 0.01                # integral gain governs rate of convergence of gyroscope biases
halfT = 0.0025           # half the sample period				
q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0
exInt = 0.0
eyInt = 0.0
ezInt = 0.0       # scaled integral error	

				   
def imu(acc,gyro):
  global q0, q1, q2, q3
  global Kp, Ki, harlT
  global exInt, eyInt, ezInt
  gx = (gyro[_x])/57.3
  gy = (gyro[_y])/57.3
  gz = (gyro[_z])/57.3
  
  ax = acc[_x]
  ay = acc[_y]
  az = acc[_z]
     
  #v_acc = math.sqrt(ax*ax+ay*ay+az*az)
  
  norm = 1.0/math.sqrt(ax*ax+ay*ay+az*az)
  ax = ax * norm
  ay = ay * norm
  az = az * norm
  
  vx = 2*(q1*q3 - q0*q2)
  vy = 2*(q0*q1 + q2*q3)
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3
  
  v_acc = ax*vx + ay*vy + az*vz
  
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);
  
  # integral error scaled integral gain
  exInt = exInt + ex*Ki
  eyInt = eyInt + ey*Ki
  ezInt = ezInt + ez*Ki
  
  
  # adjusted gyroscope measurements		用叉积误差来做PI修正陀螺零偏	
  gx = gx+ Kp*ex + exInt
  gy = gy + Kp*ey + eyInt
  gz = gz + Kp*ez + ezInt
  
  #integrate quaternion rate and normalise		四元数微分方程   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT
  
  # normalise quaternion				四元数规范化
  norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
  q0 = q0 / norm
  q1 = q1 / norm
  q2 = q2 / norm
  q3 = q3 / norm
  
  #转换为欧拉角
  pitch  = math.asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3  
  roll   = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3
  yaw    = -math.atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3
  
  return (pitch, roll, yaw, v_acc)
