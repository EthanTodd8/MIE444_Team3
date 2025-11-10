import time
import serial

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM7'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)

### Limit Variables ###
'''A = 17.78
B = 13.97
C = 76.2
D = 8.13
E = 25.4
F = 38.1'''


def transmit(data):
    '''Function to transmit data over serial connection'''
    ser.write(data.encode('ascii'))

def read_us():
    '''Function asks sensor board to take a reading of the ultrasonic sensors by sending 'u' over serial'''
    case = True
    values = []
    send = 'u'
    #print('reading_us') #Debug statement
    while case is True:
        #print('sending') #Debug statement
        ser.write(send.encode('utf-8')) #sends us cmd to Arduino
        time.sleep(0.001) #add delay
        line = ser.readline().strip().decode('ascii')
        print(line)

        if line:
            # Split using ',' as the delimiter
            values_str = line.split(',') # List to store unltrasonic sensor readings into [Back, Left, Front, Right]
            #print(values_str)
            case = False
            for i in range(len((values_str))-1):
                values.append(float(values_str[i]))
            return values
        case = True
        
def read_g():
    '''Function asks sensor board to take gyroscope reading'''
    case = True
    values = []
    send = 'g'
    #print('reading_us') #Debug statement
    while case is True:
        #print('sending') #Debug statement
        ser.write(send.encode('utf-8')) #sends us cmd to Arduino
        time.sleep(0.001) #add delay
        line = ser.readline().strip().decode('ascii')
        print(line)

        if line:
            # Split using ',' as the delimiter
            values_str = line.split(',') # List to store unltrasonic sensor readings into [Back, Left, Front, Right]
            #print(values_str)
            case = False
            for i in range(len((values_str))-1):
                values.append(float(values_str[i]))
            return values
        case = True

def move_forward():
    '''Function to move rover forward by sending 'F' '''
    transmit('F')

def move_backward():
    '''Function to move rover backward by sending 'B' '''
    transmit('B')

def move_right():
    '''Function to move rover right by sending 'R' '''
    transmit('R')

def move_left():
    '''Function to move rover left by sending 'L' '''
    transmit('L')

RUN = True
SLEEP_TIME = 0.001

while RUN:
    time.sleep(SLEEP_TIME)  # Pause Time 
    print('running')

    cmd = input('Enter char: ')
    if cmd == 'u':
        readings = read_us() #readings in format [Back, Left, Front, Right]
        print(readings)
    
    elif cmd == 'f':
        move_forward()

    elif cmd == 'r':
        move_right()

    elif cmd == 'l':
        move_left()

    elif cmd == 'b':
        move_backward()
        
    readings.clear()
