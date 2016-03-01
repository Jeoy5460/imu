#!/usr/bin/python
# -*- coding: utf-8 -*-

from collections import deque
import threading, math, time
from sys import platform as _platform
from getch import getch
import timeit
import os
if _platform == "linux" or _platform == "linux2":
    from pylab import *


import uart_task
#u_task = uart_task.UartTask()

def key_task():
    while True:
        z = getch()
        # escape key to exit
        if ord(z) == 0x70: # p
            u_task.close_uart()
            #plot(t_array, spd_array)
            exit(0)
        elif ord(z) == 0x6F: # o
            u_task.open_uart()


import ble_task
import calc_task
import multiprocessing
class FileTask():
    def __init__(self, fname=''):
        self.fname = fname
        self.d_dat = deque()

    def get_deque(self):
        if len(self.fname)!=0 and os.path.isfile(self.fname):
            fd_dat= open(self.fname, 'r')
            data = [map(float32,line.split()) for line in fd_dat]
            m_data = np.array(data,float32)
            for val in m_data[:,[0,1,2,3,4]]:
                print "mdata", val
                self.d_dat.append([val[0], val[1], (val[2], val[3], val[4])])
        else:
            print "File not existed"
        return self.d_dat
 
def run(flag,fname=''):
    if flag == 1:
        dat_task = uart_task.UartTask()
    elif flag == 2:
        dat_task = ble_task.BleTask()
    elif flag == 3:
        dat_task = FileTask(fname)
               
    if isinstance(dat_task,threading.Thread):
        print "It's a threading"
        dat_task.setDaemon(True)
    deq_data = dat_task.get_deque()


    c_task = calc_task.CalcTask()
    c_task.setDaemon(True)
    c_task.set_d_acc_gyr(deq_data)

    #t_key = multiprocessing.Process(target=key_task)
    #t_key = threading.Thread(target=key_task)
    #t_key.deamon = True

    if isinstance(dat_task,threading.Thread):
        dat_task.start()
    c_task.start()
    #t_key.start()
    while True:
        #data = tag.movement.read()[0:6]
        #print("movement: ", data)
        #tag.waitForNotifications(0.02)
        time.sleep(10000)
    #tag.disconnect()
    #del self.tag
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-b', action='store_true')
    parser.add_argument('-u', action='store_true')
    parser.add_argument('-f', nargs=1) 
    args = parser.parse_args() 
    if args.u:
        run(1)
    elif args.b:
        run(2)
    elif len(args.f):
        run(3,args.f[0])
