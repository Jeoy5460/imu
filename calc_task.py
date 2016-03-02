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
        self.dlen = 4 

        self.speed = 0.0
        self.pre_tm = 0.0
        self.sum_t = 0.0
        self.calc_init = True

        self.d_vel_tm_buf =  deque()

        self.sr = 15.0

        que_len = 100
        self.d_acc_gyr = deque()
        self.que_det = deque(maxlen = int(self.sr))
        self.q_acc_drift = deque([0]*que_len)
        self.q_vel_drift = deque([0]*que_len)
        self.q_pos_drift = deque([0]*que_len)
        self.pos = 0.0
        self.filename = ''

        self.st = 0 #0:waiting or calculating; 1:getting data; 
        self.vel = []        
        self.is_stationary = 1
        self.acc = 0.0

    def calc_speed(self, tm_stamp, g_acc):
        if (self.calc_init):
            self.pre_tm = tm_stamp
            self.speed = 0.0
            self.sum_t = 0.0
            self.acc = g_acc
            self.calc_init = False
            return (0,0)
        else:
            tm_interval =  tm_stamp - self.pre_tm

            self.speed = (9.8 * (self.acc-1))*tm_interval/1000.0 + self.speed;
            self.sum_t += tm_interval
            #self.pre_tm = timeit.default_timer()
            self.pre_tm = tm_stamp
            self.acc = g_acc
            return (tm_interval, self.speed)

    def calc_position(self, tm_interval, vel):
        self.pos += vel*tm_interval/1000.0
        return self.pos

    def calc_height(self, vel_buf):
        if len(vel_buf) > 1:
            #a = vel_buf[-1][1]/ vel_buf[-1][0]
            a = vel_buf[-1][1]/self.sum_t 
            dis = 0.0
            cur_tm = 0.0
            with open("vel.dat", 'a+') as fd:
                for val in vel_buf:
                    cur_tm += val[0]
                    dis += (val[1] - cur_tm*a)*(val[0]/1000.0)
                    print>>fd, val[1], val[1] - cur_tm*a, dis  
            return dis
        else:
            print "velocity and time buffer empty"
            return 0

    def set_d_acc_gyr(self, d_acc_gyr):
        self.d_acc_gyr = d_acc_gyr

    def get_acc_gyro(self, arr):
        """
        if len(arr) == 12:
            return [(256*arr[1]+ls[0], 256*arr[3]+ls[2], 256*arr[5]+arr[4]),
                    (256*arr[7]+ls[6], 256*arr[9]+ls[8], 256*arr[11]+arr[10]),()]
        elif len(arr) == self.dlen:
            return (arr[0], arr[1], arr[2], arr[3]) 
        else:
            print "get acc error"
            return []
        """
        pass

    def motion_detected(self, data):
        filtCutOff = 0.001;
        [b, a] = butter(1, (2*filtCutOff*1.0)/(self.sr), btype='highpass');
        acc_magFilt = filtfilt(b, a, data);
        #acc_magFilt = lfilter(b, a, lfilter(b,a,data));
        acc_magFilt = [abs(x) for x in acc_magFilt] 
        filtCutOff = 0.3;
        [b, a] = butter(1, (2*filtCutOff)*1.0/(self.sr), btype='lowpass');
        filtered = filtfilt(b, a, acc_magFilt);
        #filtered = lfilter(b, a, lfilter(b,a,acc_magFilt));

        stan = filtered < 0.05
        return stan, filtered

    
    def gait_track(self, tm_stamp, g_acc):
        #is_stationary,flt = self.motion_detected(angle)
        self.que_det.append(g_acc)
        if len(self.que_det)>= self.que_det.maxlen:
            stationary,flt = self.motion_detected(list(self.que_det))
            
            std = np.std(list(self.que_det))
            #if stationary[-1] == 1:
            if std< 0.005:
                self.is_stationary = 1
            else:
                self.is_stationary = 0
            #print std
            with open('filt.dat', 'a+') as fd:
                #fd2.write('%03f %03f %03f \n' % (g_acc,flt,is_stationary))
                #print>>fd, ('%03f %03f %03f' %(self.que_det[i], flt[i], stationary[i]))
                print>>fd, ('%03f %03f %03f' %(self.que_det[-1], std, self.is_stationary))

        if (self.st == 0):
            if (self.is_stationary):
                self.st = 0
            else:
                self.st = 1
        elif self.st == 1:
            if (not self.is_stationary):
                (interval, spd) = self.calc_speed(tm_stamp, g_acc)
                self.vel.append((interval, spd))
            else:
                if len(self.vel) != 0:
                    print "start"
                    print self.calc_height(self.vel)
                    print "end"
                    self.vel = []
                #self.que_det.clear()
                self.calc_init = True
                self.st = 0

    def oscillatory_motion(self, tm_stamp, v_acc): 
        self.q_acc_drift.append(v_acc)
        acc_hp = self.filter_drift(self.q_acc_drift)

        self.q_acc_drift.popleft()
        print "q_acc hp:", acc_hp[0] 
        tm_interval, vel = self.calc_speed(tm_stamp, acc_hp[0])

        self.q_vel_drift.append(vel)
        vel_hp = self.filter_drift(self.q_vel_drift)
        self.q_vel_drift.popleft()

        pos = self.calc_position(tm_interval, vel_hp[0])

        self.q_pos_drift.append(pos)
        pos_hp = self.filter_drift(self.q_pos_drift)
        self.q_pos_drift.popleft()
        with open("osc_motion.dat", "a+") as fd:
            print>>fd, acc_hp[0], vel_hp[0], pos_hp[0]
            print "position:", pos_hp[0]
            print "vel:", vel_hp[0]


    def filter_drift(self,q_drift):
        self.sr = 15
        filtCutOff = 0.1;
        [b, a] = butter(1, (2*filtCutOff*1.0)/(self.sr), 'high');
        print b,a
        print q_drift
        ls_hp = filtfilt(b, a, list(q_drift));
        return ls_hp

    def run(self):
       # fd2 = open("origin.txt", "w+")

        while 1:
            if len(self.d_acc_gyr) != 0:
                #tm_v:0:time stamp 1: v_acc
                tm_v_acc = self.d_acc_gyr.popleft()

                #ax = tm_v_acc[2][_x]/1000.0 - 10
                #ay = tm_v_acc[2][_y]/1000.0 - 10
                #az = tm_v_acc[2][_z]/1000.0 - 10

                #gx = (tm_v_acc[3][0]./10.0-2000);
                #gy = (tm_v_acc[3][1]/10.0-2000);
                #gz = (tm_v_acc[3][2]/10.0-2000);

                #print "acc_uart: %f %f %f" % (acc[_x], acc[_y], acc[_z])
                #pry = imu([ax,ay,az],[gx,gy,gz])
                #sum_acc = math.sqrt(ax*ax+ay*ay+az*az)
                #fd2.write('%03f %03f %03f %03f %03f %03f %03f %03f\n' % (t, pry[3], ax, ay, az, gx, gy, gz))
                
                if type(tm_v_acc[1])>1000:
                    g_acc = tm_v_acc[1]/1000.0-10
                else:
                    g_acc = tm_v_acc[1]
                angle = 0.0
                #angle = math.atan2(ax, az)+ 3.15
                self.gait_track(tm_v_acc[0], g_acc)
                #self.oscillatory_motion(tm_v_acc[0], g_acc) 

            else:
                time.sleep(0.001)

