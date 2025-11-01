import serial
import time
ser = serial.Serial('COM6', 9600, timeout=0) # Initialize COM port
s = ' '
while s:
    s = input('enter chr: ')
    ser.write(s.encode()) # writes letter to Arduino
    time.sleep(0.5) # you can also use pause(0.1)
    print(ser.readline().strip().decode('ascii')) # reads from Arduino
ser.close() # Closes connection
