import time
import serial

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM8'    # COM port identification
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

def move_right_big():
    '''Function to move rover right by sending 'R' '''
    transmit('R')

def move_right_small():
    ''' Function to move rover right be a small increment by sending 'r' '''
    transmit('r')

def move_left_big():
    '''Function to move rover left by sending 'L' '''
    transmit('L')

def move_left_small():
    ''' Function to move rover left by a small increment by sending 'l' '''
    transmit('l')
    
def stop():
    '''Function to stop rover motors from moving'''
    transmit('S')

g_ref = read_g() #reference gyrsoscope reading (assuming parallel to a wall)

def Obstacle_Avoidance_Forward(g_ref, g_current):
    #Check Difference In Reading
    g_measured = g_current[0] - g_ref[0]
    
    #Check for 3 degrees difference between reference orientation and current reading

    while g_measured > 3: #If the distance is greater than 3 degrees, then the rover is too far to the right and needs to reorient left
            move_left_small() # re-orient by small step to the left and re-calculate delta
            g_current = read_g()
            g_measured = g_current[0] - g_ref[0]

    while g_measured < -3: #If the distance is less than 3 degrees, then the rover is too far to the left and needs to reorient to the right
            move_right_small() #re-orient by small step to the right and re-calculate delta
            g_current = read_g()
            g_measured = g_current[0] - g_ref[0]

    return(g_current) # Returns corrected current orientation

def Obstacle_Avoidance_Right(g_ref_old, g_current):

    g_ref = []
    g_ref.append(g_ref_old[0] + 90) #add 90 degrees to original reference orientation to get new orientation
    
    #Check Difference In Reading
    g_measured = g_current[0] - g_ref[0]

    #Check for 3 degrees difference between reference orientation and current reading

    while g_measured > 3: #If the distance is > 3 degrees, then the rover is too far to the right and needs to reorient left
        move_left_small() #re-orient by small step to the left and re-calculate delta
        g_current = read_g()
        g_measured = g_current[0] - g_ref[0]
    
    while g_measured < -3: #If the distance is < -3 degrees, then the rover is too far to the left and needs to reorient right
        move_right_small() #re-orient by small step to the right and re-calculate delta
        g_current = read_g()
        g_measured = g_current[0] - g_ref[0]

    return(g_current, g_ref)

def Obstacle_Avoidance_Left(g_ref_old, g_current):

    g_ref = []
    g_ref.append(g_ref_old[0] - 90) #subtract 90 degrees to original reference orientation to get new orientation 
    #Check Difference In Reading
    g_measured = g_current[0] - g_ref[0]

    #Check for 3 degrees difference between reference orientation and current reading

    while g_measured > 3: #If the distance is > 3 degrees, then the rover is too far to the right and needs to reorient left
        move_left_small() #re-orient by small step to the left and re-calculate delta
        g_current = read_g()
        g_measured = g_current[0] - g_ref[0]
    
    while g_measured < -3: #If the distance is < -3 degrees, then the rover is too far to the left and needs to reorient right
        move_right_small() #re-orient by small step to the right and re-calculate delta
        g_current = read_g()
        g_measured = g_current[0] - g_ref[0]

    return(g_current, g_ref)

## Time & Counter Set-Up
SLEEP_TIME = 0.001
counter = 0

## Inititalize Gyroscope ##
g_ref = read_g() #reference gyrsoscope reading (assuming parallel to a wall)
g_readings = [] #list for gyroscope measurements 
g_readings.append(g_ref) #first reading

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

    if readings[2] > 13:
       move_forward()
       g = read_g()
       print(g)
       g_readings.append(g) # Store current counter reading
       g_new = Obstacle_Avoidance_Forward(g_ref, g_readings[counter]) #re-orient and correct current orientation if needed
       print("oriented")
       g_readings[counter] = g_new #Update current gyroscope position


    elif readings[3] > 13: 
        move_right_big()
        time.sleep(SLEEP_TIME)
        move_right_big()

        g = read_g()
        g_readings.append(g) #Store current counter reading
        g_new, g_ref_new = Obstacle_Avoidance_Right(g_ref, g_readings[counter]) #re-orient and correct current position
        p
        g_readings[counter] = g_new #Update current gyroscope position

    elif readings[1] > 13:
        transmit('L')
        time.sleep(SLEEP_TIME)
        transmit('L')

        g = read_g()
        g_readings.append(g) #Store current counter reading
        g_new, g_ref_new = Obstacle_Avoidance_Left(g_ref, g_readings[counter]) #re-orient and correct current position
        g_readings[counter] = g_new #Update current gyroscope position 
        g_ref = g_ref_new #Update g_ref 

while LZ_R: 
    time.sleep(SLEEP_TIME) # delay to not overload with readings
    counter += 1

    #Check US Sensor Readings 
    readings = read_us() #readings in format [Back, Left, Front, Right]
    print(readings)     

    if readings[2] > 10:
       transmit('F')

       g = read_g()
       g_readings.append(g) # Store current counter reading
       g_new = Obstacle_Avoidance_Forward(g_ref, g_readings[counter]) #re-orient and correct current orientation if needed
       g_readings[counter] = g_new #Update current gyroscope position      
    
    elif readings[1] > 10: 
        transmit('L')
        time.sleep(SLEEP_TIME)
        transmit('L')

        g = read_g()
        g_readings.append(g) #Store current counter reading
        g_new , g_ref_new = Obstacle_Avoidance_Left(g_ref, g_readings[counter]) #re-orient and correct current position
        g_readings[counter] = g_new #Update current gyroscope position 
        g_ref = g_ref_new #Update g_ref 

    elif readings[3] > 10:
        transmit('R')
        time.sleep(SLEEP_TIME)
        transmit('R')

        g = read_g()
        g_readings.append(g) #Store current counter reading
        g_new , g_ref_new = Obstacle_Avoidance_Right(g_ref, g_readings[counter]) #re-orient and correct current position
        g_readings[counter] = g_new #Update current gyroscope position
        g_ref = g_ref_new #Update g_ref 