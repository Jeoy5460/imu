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

        elif ord(z) == 0x6F: # o
            u_task.open_uart()


import calc_task
c_task = calc_task.CalcTask()
           
def run():
    u_task.setDaemon(True)
    c_task.setDaemon(True)
    c_task.set_d_acc_gyr(u_task.get_d_acc_gyr())
    
    
    t_key = threading.Thread(target=key_task)
    t_key.deamon = True
    
    t_key.start()
    c_task.start()
    u_task.start()
    while True:
    	time.sleep(1)
            
if __name__ == "__main__":
    run()