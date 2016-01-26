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

def run():
    p = 1
    if p == 1:
        dat_task = uart_task.UartTask()
    elif p == 2:
        dat_task = ble_task.BleTask()

    dat_task.setDaemon(True)
    deq_data = dat_task.get_deque()


    c_task = calc_task.CalcTask()
    c_task.setDaemon(True)
    c_task.set_d_acc_gyr(deq_data)

    #t_key = multiprocessing.Process(target=key_task)
    #t_key = threading.Thread(target=key_task)
    #t_key.deamon = True

    c_task.start()
    dat_task.start()
    #t_key.start()
    #dat_task.start()
    while True:
        #data = tag.movement.read()[0:6]
        #print("movement: ", data)
        #tag.waitForNotifications(0.02)
        time.sleep(10000)
    #tag.disconnect()
    #del self.tag
if __name__ == "__main__":
    run()
