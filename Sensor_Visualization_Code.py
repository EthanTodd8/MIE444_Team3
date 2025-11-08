## Ultrasonic Sensor Visualization Code
import serial
import time
#import csv
import numpy as np

ser = serial.Serial('COM9', 9600, timeout=0) # Initialize COM port

##Command Codes
read_us = 'u'
d_forward = 'f'
d_backward = 'b'
d_left = 'l'
d_right = 'r'

## Sensor Arrays

## Get Sensor Readings
case = True #Intiialize to send and receive commands

while case:
    
    ser.write(read_us.encode()) #sends us cmd to Arduino
    time.sleep(0.5) #add delay
    line = ser.readline().strip().decode('ascii')

    if not line:
        continue

    # Split using ',' as the delimiter
    all_sensors = line.split(',')
    sensor_back = all_sensors[0]
    sensor_left = all_sensors [1]
    sensor_front = all_sensors[2]
    sensor_right = all_sensors[3]
    print(all_sensors)
    print(sensor_back, sensor_left, sensor_front, sensor_right)

ser.close()
