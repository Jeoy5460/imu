#!/usr/bin/env python
# -*- coding: utf-8 -*-
import threading
from collections import deque
from imu import imu as imu
import timeit, time
import math

ACC = 0
GYRO = 1
_x = 0
_y = 1
_z = 2

class CalcTask(threading.Thread):
    def __init__(self):
        super(CalcTask, self).__init__()
        self.speed = 0.0
        self.pre_tm = 0.0
        self.sum_t = 0.0
        
        self.d_vel_tm_buf =  deque()
        
        self.d_acc_gyr = deque()
        
    def calc_speed(self, v_acc):
        global  sum_t
        if (self.pre_tm-0.0001 < 0):
            self.pre_tm = timeit.default_timer() 
        else:
            tm_sec =  timeit.default_timer() - self.pre_tm
            
            self.speed = (9.8 * (v_acc-1))*tm_sec + self.speed;
            self.sum_t += tm_sec
            self.pre_tm = timeit.default_timer()
        return self.sum_t, self.speed
    
    def set_d_acc_gyr(self, d_acc_gyr):
        self.d_acc_gyr = d_acc_gyr
    
    def get_acc_gyro(self, ls):
        if len(ls) == 12:
            return [(256*ls[1]+ls[0], 256*ls[3]+ls[2], 256*ls[5]+ls[4]),
                    (256*ls[7]+ls[6], 256*ls[9]+ls[8], 256*ls[11]+ls[10])]
        else:
            print "get acc error"
            return ()  
            
    def run(self):
        
        st_move = 0
        pre_st_move = 0
        
        start_cnt = 0
        end_cnt = 0
        data_st = 0
        calc_st = 0
        dis = 0.0
        
        tm_stamp = 0.0
        pre_tm_calc = 0.0
        
        fd2 = open("origin.txt", "w")
        
        while 1:
            
            if len(self.d_acc_gyr) != 0:
            
                [acc,gyro] = self.get_acc_gyro(self.d_acc_gyr.pop())
                
                ax = acc[_x]/1000.0 - 10
                ay = acc[_y]/1000.0 - 10
                az = acc[_z]/1000.0 - 10
                
                gx = (gyro[_x]/10.0-2000);
                gy = (gyro[_y]/10.0-2000);
                gz = (gyro[_z]/10.0-2000);
                
                
                #print "acc_uart: %f %f %f" % (acc[_x], acc[_y], acc[_z])
                #print "acc: %f %f %f" % (ax, ay, az)
                pry = imu([ax,ay,az],[gx,gy,gz])
                
                #tm_stamp += (timeit.default_timer() - pre_tm_calc)
                #pre_tm_calc = timeit.default_timer() 
                #print timeit.default_timer()
                fd2.write('%03f %03f %03f %03f %03f %03f %03f %03f %03f\n' % (timeit.default_timer(), gx, gy, gz, ax, ay, az, pry[3], math.sqrt(ax*ax+ay*ay+az*az))) 
                
                #fd2.write('%03f %03f %03f \n' % (pry[0], pry[1], pry[2]))
                #fd2.write('%03f %03f %03f \n' % (ax,ay,az))                 
                #print "pitch:%f roll:%f yaw:%f v_acc:%f" %(pry[0],pry[1],pry[2],pry[3])
                
                if (abs(gy) <= 15): #and abs(gy) < 15 and abs(gz) < 15
                    end_cnt += 1
                    start_cnt = 0
                    if end_cnt > 15:
                        pre_st_move = st_move
                        st_move = 0 
                else:
                    start_cnt += 1
                    end_cnt = 0
                    if start_cnt > 15:
                        pre_st_move = st_move
                        st_move = 1
                if 0 == pre_st_move and st_move == 1:
                    data_st = 1
                    calc_st = 0
                elif 1 == pre_st_move and 0 == st_move:
                    data_st = 0
                    calc_st = 1
                
                #data_st = 0
                #calc_st = 0            
                if 1 == data_st:
                    #print "start:", gx, gy, gz
                    t,v =  self.calc_speed(pry[3])
                    self.d_vel_tm_buf.append([t,v])
                    #print "tm:", t, "vel:", v, "v_acc:", pry[3]-1
                    #fd2.write('%03f %03f %03f %03f %03f %03f %03f %03f\n' % (t, pry[3], ax, ay, az, gx, gy, gz))  
                    
                elif 1 == calc_st:
                    
                    #print "end:", gx, gy, gz
                    l_vel = []
                    self.speed = 0
                    self.pre_tm = 0.0
                    self.sum_t = 0.0
                    if len(self.d_vel_tm_buf) != 0:
                        a = self.d_vel_tm_buf[-1][1]/ self.d_vel_tm_buf[-1][0]
                        last_tm = 0.0
                        fh = open("res.txt", "a")
                        for val in self.d_vel_tm_buf:            
                            #l_vel.append([val[0] - last_tm , val[1] - val[0]*a])
                            dis += (val[1] - val[0]*a)*(val[0] - last_tm)
                            fh.write('%03f %03f %03f \n' % (val[0], val[1], val[1] - val[0]*a )) 
                            last_tm = val[0]  
                        
                        self.d_vel_tm_buf.clear()
                        fh.close()
                        print "height:",dis
                        dis = 0.0
                        calc_st = 0
                        data_st = 0
                    else:
                        print "velocity and time buffer empty"
                            
                #fh.write('{:03f,:03f}\n'.format(ax,ay))
             
            else:
#                print "calc sleeping"
                time.sleep(0.01)
        fd2.close()
   
