#!/usr/bin/env python
from bluepy.btle import UUID, Peripheral, DefaultDelegate
import struct
import math

def _TI_UUID(val):
    return UUID("%08X-0451-4000-b000-000000000000" % (0xF0000000+val))

class SensorBase:
    # Derived classes should set: svcUUID, ctrlUUID, dataUUID
    sensorOn  = struct.pack("B", 0x01)
    sensorOff = struct.pack("B", 0x00)

    def __init__(self, periph):
        self.periph = periph
        self.service = None
        self.ctrl = None
        self.data = None

    def enable(self):
        if self.service is None:
            self.service = self.periph.getServiceByUUID(self.svcUUID)
        if self.ctrl is None:
            self.ctrl = self.service.getCharacteristics(self.ctrlUUID) [0]
        if self.data is None:
            self.data = self.service.getCharacteristics(self.dataUUID) [0]
        if self.sensorOn is not None:
            self.ctrl.write(self.sensorOn,withResponse=True)

    def read(self):
        return self.data.read()

    def disable(self):
        if self.ctrl is not None:
            self.ctrl.write(self.sensorOff)

    # Derived class should implement _formatData()

def calcPoly(coeffs, x):
    return coeffs[0] + (coeffs[1]*x) + (coeffs[2]*x*x)



class AccelerometerSensor(SensorBase):
    svcUUID  = _TI_UUID(0xAA10)
    dataUUID = _TI_UUID(0xAA11)
    ctrlUUID = _TI_UUID(0xAA12)

    def __init__(self, periph):
        SensorBase.__init__(self, periph)

    def read(self):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        x_y_z = struct.unpack('bbb', self.data.read())
        return tuple([ (val/64.0) for val in x_y_z ])

class KeypressSensor(SensorBase):
    svcUUID = UUID(0xFFE0)
    dataUUID = UUID(0xFFE1)

    def __init__(self, periph):
        SensorBase.__init__(self, periph)

    def enable(self):
        self.periph.writeCharacteristic(0x60, struct.pack('<bb', 0x01, 0x00))

    def disable(self):
        self.periph.writeCharacteristic(0x60, struct.pack('<bb', 0x00, 0x00))

class SensorTag(Peripheral):
    def __init__(self,addr):
        Peripheral.__init__(self,addr)
        # self.discoverServices()
        self.accelerometer = AccelerometerSensor(self)
        self.keypress = KeypressSensor(self)
        self.movement = MovementSensor(self)


class KeypressDelegate(DefaultDelegate):
    BUTTON_L = 0x02
    BUTTON_R = 0x01
    ALL_BUTTONS = (BUTTON_L | BUTTON_R)

    _button_desc = {
        BUTTON_L : "Left button",
        BUTTON_R : "Right button",
        ALL_BUTTONS : "Both buttons"
    }

    def __init__(self):
        DefaultDelegate.__init__(self)
        self.lastVal = 0

    def handleNotification(self, hnd, data):
        # NB: only one source of notifications at present
        # so we can ignore 'hnd'.
        print ("key deleg")
        val = struct.unpack("B", data)[0]
        down = (val & ~self.lastVal) & self.ALL_BUTTONS
        if down != 0:
            self.onButtonDown(down)
        up = (~val & self.lastVal) & self.ALL_BUTTONS
        if up != 0:
            self.onButtonUp(up)
        self.lastVal = val

    def onButtonUp(self, but):
        print ( "** " + self._button_desc[but] + " UP")

    def onButtonDown(self, but):
        print ( "** " + self._button_desc[but] + " DOWN")

class MovementSensor(SensorBase):
    svcUUID  = UUID(0xFA00)
    dataUUID = UUID(0xFA04)
    ctrlUUID = UUID(0xFA03)

    def __init__(self, periph):
        SensorBase.__init__(self, periph)

    def enable(self):
        if self.service is None:
            self.periph.writeCharacteristic(49, struct.pack('<bb', 0x01, 0x00))
            #for uuids,val in  (self.periph.discoverServices()):
            #print type(self.periph.discoverServices())
                #print ("srv: ",srv)
            srvs = self.periph.discoverServices()
            #    print (UUID(0xFA00))
            uuid = UUID(0xFA00)
            self.service = srvs[uuid]

            print srvs[uuid]

            #self.service = self.periph.getServiceByUUID(self.svcUUID)
        if self.ctrl is None:
            self.ctrl = self.service.getCharacteristics(self.ctrlUUID) [0]
        if self.data is None:
            self.data = self.service.getCharacteristics(self.dataUUID) [0]
        if self.sensorOn is not None:
            self.ctrl.write(self.sensorOn,withResponse=True)

    def read(self):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        x_y_z = struct.unpack('hhhhhhh', self.data.read())
        return tuple([ (val) for val in x_y_z ])


from collections import deque
class MovementDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.d_movement = deque()


    def handleNotification(self, hnd, data):
        # NB: only one source of notifications at present
        # so we can ignore 'hnd'.
#        print ("deleg:",data)
        data =  struct.unpack('HHHHHHH', data)
        ls = [(data[0] + data[1]*256*256), (data[2]), (data[3], data[4], data[5])]
        with open ('up.dat', 'a+') as fd:
            print>>fd, ls[0], ls[1], data[3], data[4], data[5]
        self.d_movement.append(ls)
    def get_deque(self):
        return self.d_movement
import threading

class BleTask(threading.Thread):
    def __init__(self):
        super(BleTask, self).__init__()
#        self.host = "78:A5:04:86:DD:24"
#        self.host = "F4:B8:5E:EE:66:6F"
        self.host = "D4:F5:13:77:1A:7C"
        #self.host = "C4:BE:84:05:7C:E0"
        self.tag = SensorTag(self.host)
        self.d_acc_gro = deque()
        self.delegate = MovementDelegate()
        self.tag.setDelegate(self.delegate)
        self.tag.movement.enable()

    def get_deque(self):
        return self.delegate.d_movement

    def run(self):
        while True:
            self.tag.waitForNotifications(1)
        self.tag.disconnect()
        del self.tag


if __name__ == "__main__":
    import time
    import sys
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('host', action='store',help='MAC of BT device')
    parser.add_argument('-n', action='store', dest='count', default=0,
            type=int, help="Number of times to loop data")
    parser.add_argument('-t',action='store',type=float, default=5.0, help='time between polling')
    parser.add_argument('-T','--temperature', action="store_true",default=False)
    parser.add_argument('-A','--accelerometer', action='store_true',
            default=False)
    parser.add_argument('-H','--humidity', action='store_true', default=False)
    parser.add_argument('-M','--magnetometer', action='store_true',
            default=False)
    parser.add_argument('-B','--barometer', action='store_true', default=False)
    parser.add_argument('-G','--gyroscope', action='store_true', default=False)
    parser.add_argument('-K','--keypress', action='store_true', default=False)
    parser.add_argument('--all', action='store_true', default=False)

    arg = parser.parse_args(sys.argv[1:])

    print('Connecting to ' + arg.host)
    tag = SensorTag(arg.host)

    # Enabling selected sensors
    #if arg.temperature or arg.all:
    #    tag.IRtemperature.enable()
    #if arg.humidity or arg.all:
    #    tag.humidity.enable()
    #if arg.barometer or arg.all:
    #    tag.barometer.enable()
    #if arg.accelerometer or arg.all:
    #    tag.accelerometer.enable()
    #if arg.magnetometer or arg.all:
    #    tag.magnetometer.enable()
    #if arg.gyroscope or arg.all:
    #    tag.gyroscope.enable()
    #if arg.keypress or arg.all:
    #    tag.keypress.enable()
    #    tag.setDelegate(KeypressDelegate())
    if arg.all:
        tag.setDelegate( MovementDelegate())
        tag.movement.enable()

    # Some sensors (e.g., temperature, accelerometer) need some time for initialization.
    # Not waiting here after enabling a sensor, the first read value might be empty or incorrect.
    time.sleep(1.0)

    counter=1
    while True:
       #if arg.temperature or arg.all:
       #    print('Temp: ', tag.IRtemperature.read())
       #if arg.humidity or arg.all:
       #    print("Humidity: ", tag.humidity.read())
       #if arg.barometer or arg.all:
       #    print("Barometer: ", tag.barometer.read())
       #if arg.accelerometer or arg.all:
       #    print("Accelerometer: ", tag.accelerometer.read())
       #if arg.magnetometer or arg.all:
       #    print("Magnetometer: ", tag.magnetometer.read())
       #if arg.gyroscope or arg.all:
       #    print("Gyroscope: ", tag.gyroscope.read())
       if counter >= arg.count and arg.count != 0:
           break
       if arg.all:
           #print("movement: ", tag.movement.read())
           pass
       counter += 1
       #tag.waitForNotifications(arg.t)
       tag.waitForNotifications(0.02)

    tag.disconnect()
    del tag
