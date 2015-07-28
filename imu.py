# -*- coding: utf-8 -*-
import serial
from collections import deque
import threading, math, time
from sys import platform as _platform
from getch import getch
import timeit
if _platform == "linux" or _platform == "linux2":
    from pylab import * 

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

speed = 0.0
distance = 0.0
pre_tm = 0.0
sum_t = 0.0

def calc_height(v_acc):
    global speed, distance, pre_tm, sum_t
    if (pre_tm-0.0001 < 0):
        pre_tm = timeit.default_timer() 
    else:
        tm_sec =  timeit.default_timer() - pre_tm
        
        speed = (9.8 * (v_acc-0.9998))*tm_sec + speed;
        distance += speed*tm_sec;
        sum_t += tm_sec
        pre_tm = timeit.default_timer()
    return distance,sum_t,speed
    
    
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
ser = serial.Serial()
ser_flag = 1

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
        lengh = byte
        st = 3
    elif 3 == st:
        if cnt  > 1:
            cnt -= 1
            acc_gyr.append(byte)
        else:
            if byte == (sum(acc_gyr)&0xFF):
                if len(acc_gyr) != 0:
                    d_acc_gyr.append(acc_gyr)
                else:
                    #ser.close()
                    #to do why would this happen
                    print "last byte & acc_gyr:", byte,lengh
            st = 0
  
def get_acc_gyro(ls):
    if len(ls) == 12:
        return [(256*ls[1]+ls[0], 256*ls[3]+ls[2], 256*ls[5]+ls[4]),
                (256*ls[7]+ls[6], 256*ls[9]+ls[8], 256*ls[11]+ls[10])]
    else:
        print "get acc error"
        return ()
             
def key_task():
    global ser_flag
    while True:
        z = getch()
        # escape key to exit
        if ord(z) == 0x70: # p
           ser_flag = 0
            #plot(t_array, spd_array)

        elif ord(z) == 0x6F: # o
            ser_flag = 1
            if not ser.isOpen():
               ser.open()
 

d_vel_tm_buf =  deque()
def calc_task():
    sample_cnt = 0
    sum_vcc = 0
    global d_acc_gyr, pre_tm
    cali_cnt = 0
    cali_ax = 0.0
    cali_ay = 0.0
    cali_az = 0.0
    
    cali_gx = 0.0
    cali_gy = 0.0
    cali_gz = 0.0
    fh = open("res.txt", "w")
    while 1:
        
        if len(d_acc_gyr) != 0:
            
            [acc,gyro] = get_acc_gyro(d_acc_gyr.pop())
            
            ax = acc[_x]/1000.0 - 10
            ay = acc[_y]/1000.0 - 10
            az = acc[_z]/1000.0 - 10
            
            gx = (gyro[_x]/10.0-2000);
            gy = (gyro[_y]/10.0-2000);
            gz = (gyro[_z]/10.0-2000);
            
            #sample_cnt+=1;
            #print "sample rate:", ((sample_cnt)/(time.time() - start_time))
            
            #print "acc_uart: %f %f %f" % (acc[_x], acc[_y], acc[_z])
            #print "acc:",ax, ay, az
            #print "gyro:", gx, gy, gz
            pry = imu([ax,ay,az],[gx,gy,gz])
            
            h,t,v =  calc_height(pry[3])
            
            d_vel_tm_buf.append([t,v])
            l_vel = []
            if (gx < 15 and gy < 15 and gz < 15 and len(d_vel_tm_buf) >100 ):

                a = d_vel_tm_buf[-1][1]/ d_vel_tm_buf[0][-1]
                last_tm = 0.0
                for val in d_vel_tm_buf:            
                    l_vel.append([val[0] - last_tm , val[1] - val[1]*a])
                    last_tm = val[0]
                for val in l_vel:
                    dis += val[1]*val[0]
                d_vel_tm_buf.clear()
                print dis
                    
            
            #print "pitch:%f roll:%f yaw:%f v_acc:%f" %(pry[0],pry[1],pry[2],pry[3])
            #sum_vcc += pry[3]
            #print "vcc:", sum_vcc/sample_cnt, sample_cnt
            
            #h,t,v =  calc_height(pry[3])
            #if cali_cnt < 100:
            #v_acc = math.sqrt(ax*ax + ay*ay + az*az)
            
            #[spd, t] = calc_v(pry[3])
            #t_array.append(t)
            #spd_array.append(spd)
            
            #print t, h, v, pry[3]
            fh.write('%03f %03f %03f %03f\n' % (t, h, v, pry[3])) 
            #fh.write('{:03f,:03f}\n'.format(ax,ay))
       
            
            
        else:
            time.sleep(0.01)
    fh.close()
    
def uart_task():
    global ser
    global ser_flag
    if _platform == "linux" or _platform == "linux2":
       _port='/dev/ttyS0'
    elif _platform == "win32":
       _port = 'com1'
    ser.port = _port
    ser.baudrate=115200
    #ser = serial.Serial(
        #port='/dev/ttyUSB1',
        #port=_port,
        #baudrate=115200,
        #parity=serial.PARITY_ODD,
        #stopbits=serial.STOPBITS_TWO,
        #bytesize=serial.SEVENBITS
    #)
    #ser.close()
    ser.open()
    #ser.isOpen()

    print 'Enter your commands below.\r\nInsert "exit" to leave the application.'
    while 1 :
        if ser.isOpen():
            c = ser.read(1)
            parse_uart(ord(c))
            
            if ser_flag == 0:
                ser.close()
                print "uart close"
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
    t_key = threading.Thread(target=key_task)
    t_key.start()
            
if __name__ == "__main__":
    run()
