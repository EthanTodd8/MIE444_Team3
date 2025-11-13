import time
import serial

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM7'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)

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
        #print(line)

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
        #print(line)

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
    
def stop():
    '''Function to stop rover motors from moving'''
    transmit('S')

## Time & Counter Set-Up
SLEEP_TIME = 0.001
counter = 0

## Determine whether left or right wall-following
readings = read_us() #readings in format [Back, Left, Front, Right]
print(readings)

if readings[3]> readings[1]:  #if RIGHT distance > LEFT distance, follow the LEFT wall
    LZ_R = False # Follow Left Wall
    LZ_L = True
    print(LZ_R, LZ_L)

else: 
    LZ_R = True # Follow Right Wall
    LZ_L = False
    print(LZ_R, LZ_L)

### Previous logic sequence with small steps, run this if the rover has a move forward function that stops after a time delay ###

while LZ_L: 
    time.sleep(SLEEP_TIME) # delay to not overload with readings
    counter += 1

    #Check US Sensor Readings 
    readings = read_us() #readings in format [Back, Left, Front, Right]
    print(readings)     

    if readings[2] > 10:
       transmit('F')

    elif readings[3] > 10: 
        transmit('R')
        time.sleep(SLEEP_TIME)
        transmit('R')

    elif readings[1] > 10:
        transmit('L')
        time.sleep(SLEEP_TIME)
        transmit('L')
    
    elif readings[2] < 10 and readings[3] < 10 and readings [1] < 10:
        LZ_L = False # Stop Loop
        path_1 = print ('Rover is at bottom right portion of the map, use path_1 to loading zone.')
    