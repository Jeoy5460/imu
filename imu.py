# -*- coding: utf-8 -*-
import serial
from collections import deque
import threading, math, time
from sys import platform as _platform

Kp = 2.0                 # proportional gain governs rate of convergence to accelerometer/magnetometer
Ki = 0.01                # integral gain governs rate of convergence of gyroscope biases
halfT = 0.0025           # half the sample period				
q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0;
exInt = 0.0
eyInt = 0.0
ezInt = 0.0;        # scaled integral error	

				   
def imu(acc,gyro):
  global q0, q1, q2, q3
  global Kp, Ki, harlT
  global exInt, eyInt, ezInt
  gx = (gyro[_x]/10.0-2000)*57.3;
  gy = (gyro[_y]/10.0-2000)*57.3;
  gz = (gyro[_z]/10.0-2000)*57.3;
  
  ax = acc[_x]
  ay = acc[_x]
  az = acc[_x]
  
  norm = 1.0/math.sqrt(ax*ax+ay*ay+az*az)
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm; 
  
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
  
  ex = (ay*vz - az*vy);
  ey = (az*vx - ax*vz);
  ez = (ax*vy - ay*vx);
  
  # integral error scaled integral gain
  exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;
  
  
  # adjusted gyroscope measurements		用叉积误差来做PI修正陀螺零偏	
  gx = gx+ Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;
  
  #integrate quaternion rate and normalise		四元数微分方程   
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT; 
  
  # normalise quaternion				四元数规范化
  norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  #转换为欧拉角
  pitch  = math.asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3  
  roll   = math.atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3
  yaw    = -math.atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3
  return (pitch, roll, yaw)

cnt = 0
lengh = 0
st = 0
acc_gyr = []
d_acc_gyr = deque()
ACC = 0
GYRO = 1
_x = 0
_y = 1
_z = 2

def parse_uart(byte):
    global cnt
    global lengh
    global st
    global acc_gyr
    global d_acc_gyr
    if 0 == st:
        if byte == 0xAA:
            cnt = 0
            chk_sum = 0
            acc_gyr = []
            st = 1
        else:
            st = 0
    elif 1 == st:
        if byte == 0x55:
            st = 2
        else:
            st = 0
    elif 2 == st:
        cnt = byte
        
        st = 3
    elif 3 == st:
        if cnt  > 1:
            cnt -= 1
            acc_gyr.append(byte)
        else:
            if byte == (sum(acc_gyr)&0xFF):
                d_acc_gyr.append(acc_gyr)
                #print acc_gyr
                acc_gyr = []
            st = 0

 
def get_acc_gyro(ls):
    if len(ls) == 12:
        return [(256*ls[1]+ls[0], 256*ls[3]+ls[2], 256*ls[5]+ls[4]),
                (256*ls[7]+ls[6], 256*ls[9]+ls[8], 256*ls[11]+ls[10])]
    else:
        print "get acc error"
        return ()
        
def calc_task():
    global d_acc_gyr
    
    while 1:
        if len(d_acc_gyr) != 0:
            
            [acc,gyro] = get_acc_gyro(d_acc_gyr.pop())
            
            ax = acc[_x]/1000.0 - 10
            ay = acc[_y]/1000.0 - 10
            az = acc[_z]/1000.0 - 10
            
            print "acc_uart: %f %f %f" % (acc[_x], acc[_y], acc[_z])
            print "acc:",ax, ay, az
            print "pitch:%f roll:%f yaw:%f" %(imu(acc,gyro)[0],imu(acc,gyro)[1],imu(acc,gyro)[2])
        else:
            time.sleep(0.01)
    
def uart_task():
    if _platform == "linux" or _platform == "linux2":
       _port='/dev/ttyUSB1'
    elif _platform == "win32":
       _port = 'com10'
    ser = serial.Serial(
        #port='/dev/ttyUSB1',
        port=_port,
        baudrate=115200,
        #parity=serial.PARITY_ODD,
        #stopbits=serial.STOPBITS_TWO,
        #bytesize=serial.SEVENBITS
    )
    #ser.close()
    #ser.open()
    ser.isOpen()

    print 'Enter your commands below.\r\nInsert "exit" to leave the application.'
    while 1 :
        c = ser.read(1)
        parse_uart(ord(c))
        #if len(d_acc_gyr) != 0:
            #acc_gyro = []
            #[tmp_acc,tmp_gyro] = get_acc_gyro(d_acc_gyr.pop())
            #print tmp_acc
            #print tmp_gyro
def run():
	t_u = threading.Thread(target=uart_task)
	t_u.start()
	t_c = threading.Thread(target=calc_task)
	t_c.start()
            
if __name__ == "__main__":
    run()
