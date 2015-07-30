#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
from collections import deque
import serial
import time
from sys import platform as _platform

class UartTask(threading.Thread):
    def __init__(self):
        super(UartTask, self).__init__()
        self.ser = serial.Serial()
        
        self.ser_flag = 1
        
        self.acc_gyr = []
        self.d_acc_gyr = deque()
        
        self.cnt = 0
        self.lengh = 0
        self.chk_sum = 0
        
        self.st = 0
        
    def parse_uart(self, byte):

        if 0 == self.st:
            if byte == 0xAA:
                self.cnt = 0
                self.chk_sum = 0
                self.acc_gyr = []
                self.st = 1
            else:
                self.st = 0
        elif 1 == self.st:
            if byte == 0x55:
                self.st = 2
            else:
                self.st = 0
        elif 2 == self.st:
            self.cnt = byte
            self.lengh = byte
            self.st = 3
        elif 3 == self.st:
            if self.cnt  > 1:
                self.cnt -= 1
                self.acc_gyr.append(byte)
            else:
                if byte == (sum(self.acc_gyr)&0xFF):
                    if len(self.acc_gyr) != 0:
                        self.d_acc_gyr.append(self.acc_gyr)
                        self.acc_gyr = []
                        
                    else:
                        #ser.close()
                        #to do why would this happen
                        print "last byte & acc_gyr:", byte,lengh
                self.st = 0
                
    def get_d_acc_gyr(self):
        return self.d_acc_gyr
      
    def close_uart(self):
        self.ser_flag = 0

    def open_uart(self):
        if not self.ser.isOpen():
            self.ser_flag = 1
            self.ser.open()
        
    def run(self):
        _port = '/dev/ttyUSB0'
        if _platform == "linux" or _platform == "linux2":
           _port='/dev/ttyUSB0'
        elif _platform == "win32":
           _port = 'com1'
        self.ser.port = _port
        self.ser.baudrate=115200
        #ser = serial.Serial(
            #port='/dev/ttyUSB1',
            #port=_port,
            #baudrate=115200,
            #parity=serial.PARITY_ODD,
            #stopbits=serial.STOPBITS_TWO,
            #bytesize=serial.SEVENBITS
        #)
        #ser.close()
        try:
            self.ser.open()
            #ser.isOpen()
        except :
            print "uart is not open"
        print 'move to start'
        while 1 :
            if self.ser.isOpen():
                c = self.ser.read(1)
                self.parse_uart(ord(c))
                
                if self.ser_flag == 0:
                    self.ser.close()
                    print "uart close"
            else:
#                print "uart sleeping"
                time.sleep(1)

