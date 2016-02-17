#!/usr/bin/env python
# -*- coding: utf-8 -*-
import threading
from collections import deque
from imu import imu as imu
import timeit, time
import math
import numpy as np
from scipy.signal import butter, lfilter,filtfilt
import os

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
        self.calc_init = True

        self.d_vel_tm_buf =  deque()

        self.d_acc_gyr = deque()
        self.que_det = deque([0]*100)
        self.filename = ''

        self.st = 0 #0:waiting or calculating; 1:getting data; 
        self.vel = []        

    def calc_speed(self, tm_stamp, g_acc):
        if (self.calc_init):
            self.pre_tm = tm_stamp
            self.speed = 0.0
            self.sum_t = 0.0
            self.calc_init = False
            return (0,0)
        else:
            tm_interval =  tm_stamp - self.pre_tm

            self.speed = (9.8 * (g_acc-1))*tm_interval/1000 + self.speed;
            self.sum_t += tm_interval
            #self.pre_tm = timeit.default_timer()
            self.pre_tm = tm_stamp
            return (tm_interval, self.speed)

    def calc_height(self, vel_buf):
        if len(vel_buf) > 1:
            #a = vel_buf[-1][1]/ vel_buf[-1][0]
            a = vel_buf[-1][1]/self.sum_t 
            dis = 0.0
            cur_tm = 0.0
            for val in vel_buf:
                cur_tm += val[0]
                dis += (val[1] - cur_tm*a)*(val[0]/1000.0)
            #print dis
            return dis
        else:
            return 0
            print "velocity and time buffer empty"

    def set_d_acc_gyr(self, d_acc_gyr):
        self.d_acc_gyr = d_acc_gyr

    def get_acc_gyro(self, ls):
        if len(ls) == 12:
            return [(256*ls[1]+ls[0], 256*ls[3]+ls[2], 256*ls[5]+ls[4]),
                    (256*ls[7]+ls[6], 256*ls[9]+ls[8], 256*ls[11]+ls[10]),()]
        elif len(ls) == 3:
            return (ls[0],ls[1],ls[2]) 
            
        elif len(ls) == 2:
            return (ls[0]), (ls[1])
        else:
            print "get acc error"
            return []

    def motion_detected(self, data):
        sr = 15.0
        filtCutOff = 0.05;
        self.que_det.append(data)
        #print self.que_det
        [b, a] = butter(1, (2*filtCutOff)/(sr), 'high');
        acc_magFilt = filtfilt(b, a, list(self.que_det));

        acc_magFilt = [abs(x) for x in acc_magFilt] 
        filtCutOff = 0.5;
        [b, a] = butter(1, (2*filtCutOff)/(sr), 'low');
        filtered = filtfilt(b, a, acc_magFilt);
        stan = filtered < 0.09
        self.que_det.popleft()
        #return all(x == True for x in stan)
        return stan[-1], filtered[-1]
        
    def process_data(self, tm_stamp, g_acc):
        is_stationary,flt = self.motion_detected(g_acc)
        #fd2.write('%03f %03f %03f \n' % (g_acc,flt,is_stationary))
        
        if (self.st == 0):
            if (is_stationary):
                self.st = 0
            else:
                self.st = 1
        elif self.st == 1:
            if (not is_stationary):
                (interval, spd) = self.calc_speed(tm_stamp, g_acc)
                #print 't & speed', interval, g_acc 
                self.vel.append((interval, spd))
            else:
                if len(self.vel) != 0:
                    #print "start"
                    self.calc_height(self.vel)
                    #print "end"
                    self.vel = []
                self.calc_init = True
                self.st = 0
 
    def run(self):
       # fd2 = open("origin.txt", "w+")


        while 1:
            if len(self.d_acc_gyr) != 0:
                #tm_v:0:time stamp 1: v_acc
                tm_v = self.get_acc_gyro(self.d_acc_gyr.popleft())

                #ax = acc[_x]/1000.0 - 10
                #ay = acc[_y]/1000.0 - 10
                #az = acc[_z]/1000.0 - 10

                #gx = (gyro[_x]/10.0-2000);
                #gy = (gyro[_y]/10.0-2000);
                #gz = (gyro[_z]/10.0-2000);

                #print "acc_uart: %f %f %f" % (acc[_x], acc[_y], acc[_z])
                #pry = imu([ax,ay,az],[gx,gy,gz])
                #sum_acc = math.sqrt(ax*ax+ay*ay+az*az)
                #fd2.write('%03f %03f %03f %03f %03f %03f %03f %03f\n' % (t, pry[3], ax, ay, az, gx, gy, gz))
                
                v_acc = tm_v[1]/1000.0-10
                print v_acc
                self.process_data(tm_v[0], v_acc)

            else:
                time.sleep(0.001)

