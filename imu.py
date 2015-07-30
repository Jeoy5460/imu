#!/usr/bin/python
# -*- coding: utf-8 -*-

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
  #return (pitch, roll, yaw, v_acc)
  return (q0, q1, q2, q3)

speed = 0.0
pre_tm = 0.0
sum_t = 0.0

def calc_speed(v_acc):
    global speed, distance, pre_tm, sum_t
    if (pre_tm-0.0001 < 0):
        pre_tm = timeit.default_timer() 
    else:
        tm_sec =  timeit.default_timer() - pre_tm
        
        speed = (9.8 * (v_acc-1))*tm_sec + speed;
        sum_t += tm_sec
        pre_tm = timeit.default_timer()
    return sum_t,speed
    
ACC = 0
GYRO = 1
_x = 0
_y = 1
_z = 2

def get_acc_gyro(ls):
    if len(ls) == 12:
        return [(256*ls[1]+ls[0], 256*ls[3]+ls[2], 256*ls[5]+ls[4]),
                (256*ls[7]+ls[6], 256*ls[9]+ls[8], 256*ls[11]+ls[10])]
    else:
        print "get acc error"
        return ()

import uart_task    
u_task = uart_task.UartTask()  
  
def key_task():
    while True:
        z = getch()
        # escape key to exit
        if ord(z) == 0x70: # p
           u_task.close_uart()
            #plot(t_array, spd_array)

        elif ord(z) == 0x6F: # o
            u_task.open_uart()

d_vel_tm_buf =  deque()

def calc_task():
    sample_cnt = 0
    sum_vcc = 0
    global d_acc_gyr, pre_tm, speed,sum_t
    cali_cnt = 0
    
    st_move = 0
    pre_st_move = 0
    
    start_cnt = 0
    end_cnt = 0
    data_st = 0
    calc_st = 0
    dis = 0.0
    fd2 = open("origin.txt", "w")
    
    tm_stamp = 0.0
    pre_tm_calc = 0.0
    
    while 1:
        
        if len(u_task.get_d_acc_gyr()) != 0:
            
            [acc,gyro] = get_acc_gyro(u_task.get_d_acc_gyr().pop())
            
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
            
            #tm_stamp += (timeit.default_timer() - pre_tm_calc)
            #pre_tm_calc = timeit.default_timer() 
            #print timeit.default_timer()
            fd2.write('%03f %03f %03f %03f %03f %03f %03f %03f %03f %03f %03f\n' % (timeit.default_timer(), gx, gy, gz, ax, ay, az, pry[0],pry[1],pry[2],pry[3])) 
            
            #print "pitch:%f roll:%f yaw:%f v_acc:%f" %(pry[0],pry[1],pry[2],pry[3])
            
            if (abs(gx) <= 50): #and abs(gy) < 15 and abs(gz) < 15
                end_cnt += 1
                start_cnt = 0
                if end_cnt > 100:
                    pre_st_move = st_move
                    st_move = 0 
            else:
                start_cnt += 1
                end_cnt = 0
                if start_cnt > 50:
                    pre_st_move = st_move
                    st_move = 1
            if 0 == pre_st_move and st_move == 1:
                data_st = 1
                calc_st = 0
            elif 1 == pre_st_move and 0 == st_move:
                data_st = 0
                calc_st = 1
            
            data_st = 0
            calc_st = 0            
            if 1 == data_st:
                #print "start:", gx, gy, gz
                t,v =  calc_speed(pry[3])
                d_vel_tm_buf.append([t,v])
                print "tm:", t, "vel:", v, "v_acc:", pry[3]-1
                #fd2.write('%03f %03f %03f %03f %03f %03f %03f %03f\n' % (t, pry[3], ax, ay, az, gx, gy, gz))  
                
            elif 1 == calc_st:
                
                #print "end:", gx, gy, gz
                l_vel = []
                speed = 0
                pre_tm = 0.0
                sum_t = 0.0
                a = d_vel_tm_buf[-1][1]/ d_vel_tm_buf[-1][0]
                #print "lean", a
                last_tm = 0.0
                fh = open("res.txt", "a")
                for val in d_vel_tm_buf:            
                    #l_vel.append([val[0] - last_tm , val[1] - val[0]*a])
                    dis += (val[1] - val[0]*a)*(val[0] - last_tm)
                    fh.write('%03f %03f %03f \n' % (val[0], val[1], val[1] - val[0]*a )) 
                    last_tm = val[0]  
                
                
                d_vel_tm_buf.clear()
                fh.close()
                print "height:",dis
                dis = 0.0
                calc_st = 0
                data_st = 0
                    
            #sum_vcc += pry[3]
            #print "vcc:", sum_vcc/sample_cnt, sample_cnt
            
            #h,t,v =  calc_height(pry[3])
            #if cali_cnt < 100:
            #v_acc = math.sqrt(ax*ax + ay*ay + az*az)
            
            #[spd, t] = calc_v(pry[3])
            #t_array.append(t)
            #spd_array.append(spd)
            
            #print t, h, v, pry[3]
            #fh.write('%03f %03f %03f %03f\n' % (t, h, v, pry[3])) 
            #fh.write('{:03f,:03f}\n'.format(ax,ay))
       
            
            
        #else:
            #time.sleep(0.01)
    fd2.close()
    


def run():

    
    u_task.deamon = True
    u_task.start()
    
    t_c = threading.Thread(target=calc_task)
    t_c.deamon = True
    t_c.start()
    t_key = threading.Thread(target=key_task)
    t_key.deamon = True
    t_key.start()
    while True:
    	time.sleep(1)
            
if __name__ == "__main__":
    run()
