import time
import serial

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)

### Limit Variables ###
A = 7
B = 5.5
C = 30
D = 3.2
E = 10
F = 15


def transmit(data): 
    '''Function to transmit data over serial connection'''
    ser.write(data.encode('ascii'))
    
def readUs():
    '''Function asks sensor board to take a reading of the ultrasonic sensors by sending 'u' over serial'''
    case = True
    Values = []
    while case == True:
        ser.write('u'.encode('ascii')) #sends us cmd to Arduino
        time.sleep(0.5) #add delay
        line = ser.readline().strip().decode('ascii')
        print(line)

        if line:
            # Split using ',' as the delimiter
            Values_str = line.split(',') # List to store unltrasonic sensor readings into [Back, Left, Front, Right]
            print(Values_str)
            case = False
            for i in range(len((Values_str))-1):
                Values.append(float(Values_str[i]))
            return Values
            
        else:
            case = True
    
def MoveForward():
    '''Function to move rover forward by sending 'F' '''
    transmit('F')

def MoveBackward():
    '''Function to move rover backward by sending 'B' '''
    transmit('B')    

def MoveRight():
    '''Function to move rover right by sending 'R' '''
    transmit('R')
    
def MoveLeft():
    '''Function to move rover left by sending 'L' '''
    transmit('L')
    
    
readings = readUs()
print(readings)
    
if (readings[2]<A) and (readings[1]<B) and (readings[3]<B):
    MoveRight()
elif (readings[2] > C):
    MoveForward()
elif (readings[1] < D):
    MoveLeft()
elif (readings[3] < D):
    MoveRight()
elif (readings[2] > E):
    MoveForward()
elif (readings[1] > F) & (readings[2] > F):
    MoveForward()
elif (readings[3] > readings[1]) & (readings[2] > (readings[2])):
    MoveLeft()
elif (readings[1] > readings[3]) & (readings[1] > (readings[2])):
    MoveRight()
else:
    MoveForward()
readings.clear()
