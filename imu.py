import serial
from collections import deque
import threading


cnt = 0
lengh = 0
st = 0
acc_gyr = []
d_acc_gyr = deque()
ACC = 0
GYRO = 1
_x = 0
_y = 1
_y = 2

def parse_uart(byte):
    global cnt
    global lengh
    global st
    global acc_gyr
    global d_acc_gyr
    if 0 == st:
        if byte == 0xAA:
            cnt = 0
            chk_sum = 0
            acc_gyr[:] = []
            st = 1
        else:
            st = 0
    elif 1 == st:
        if byte == 0x55:
            st = 2
        else:
            st = 0
    elif 2 == st:
        cnt = byte
        st = 3
    elif 3 == st:
        if cnt  > 1:
            cnt -= 1
            acc_gyr.append(byte)
        elif byte == (sum(acc_gyr)&0xFF):
                d_acc_gyr.append(acc_gyr)
                acc_gyr = []
                st = 1
                       
    
#def imu():

def get_acc_gyro(ls):
    if len(ls) == 12:
        return [(256*ls[1]+ls[0], 256*ls[3]+ls[2], 256*ls[5]+ls[4]),
                (256*ls[7]+ls[6], 256*ls[9]+ls[8], 256*ls[11]+ls[10])]
    else:
        print "get acc error"
        return ()
#def calc_task():
    
def uart_task():
    ser = serial.Serial(
        #port='/dev/ttyUSB1',
        port='com10',
        baudrate=115200,
        #parity=serial.PARITY_ODD,
        #stopbits=serial.STOPBITS_TWO,
        #bytesize=serial.SEVENBITS
    )
    #ser.close()
    #ser.open()
    ser.isOpen()

    print 'Enter your commands below.\r\nInsert "exit" to leave the application.'
    while 1 :
        c = ser.read(1)
        parse_uart(ord(c))
        if len(d_acc_gyr) != 0:
            acc_gyro = []
            [acc,gyro] = get_acc_gyro(d_acc_gyr.pop())
            print acc
            print gyro
def run():
    t_u = threading.Thread(target=uart_task)
    t_u.start()
    #t_c = threading.Thread(target=uart_task)
            
if __name__ == "__main__":
    run()
