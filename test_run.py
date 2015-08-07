#!/usr/bin/python
# -*- coding: utf-8 -*-

from collections import deque
import threading, math, time
from sys import platform as _platform
from getch import getch
import timeit
if _platform == "linux" or _platform == "linux2":
    from pylab import * 


import uart_task    
u_task = uart_task.UartTask()  
  
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


import calc_task
import multiprocessing
c_task = calc_task.CalcTask()
           
import  ble_task 
#b_task = ble_task.BleTask()
def run():

    host = "78:A5:04:86:DD:24"
    tag = ble_task.SensorTag(host)
    ble_acc_gro = deque()
    tag.movement.enable()

    #u_task.setDaemon(True)
   # b_task.setDaemon(True)

    c_task.setDaemon(True)
    #c_task.set_d_acc_gyr(u_task.get_d_acc_gyr())
    c_task.set_d_acc_gyr(ble_acc_gro)
    
    
#    t_key = multiprocessing.Process(target=key_task)
    #t_key = threading.Thread(target=key_task)
#    t_key.deamon = True
    c_task.start()
    #u_task.start()
#    t_key.start()
    #b_task.start()
    print "here"
    while True:
        data = tag.movement.read()[0:6]
        ble_acc_gro.append(data)
        print("movement: ", data)
        tag.waitForNotifications(0.02)
#        time.sleep(1)
    tag.disconnect()
    del self.tag
            
if __name__ == "__main__":
    run()
