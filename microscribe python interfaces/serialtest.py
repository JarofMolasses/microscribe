import serial
import time
import multiprocessing as mp

ser = serial.Serial('COM8',57600,timeout=5)

if(not ser.is_open):
    ser.open()

ser.flush()

while(True):    
    val = input("Enter string to send")
    commandchar = val[0]
    ser.write(commandchar.encode('utf-8'))

    serial_rx= ser.readline()
    data = str(serial_rx[0:len(serial_rx)-2].decode("utf-8"))
    print(serial_rx.decode('utf-8'))
