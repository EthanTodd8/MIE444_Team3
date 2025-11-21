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
            values_str = line.split(',') # List to store ultrasonic sensor readings into [Back, Left, Front, Right, BlockSensor]
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
            values_str = line.split(',') 
            #print(values_str)
            case = False
            for i in range(len((values_str))-1):
                values.append(float(values_str[i]))
            if values[0] == 0.0 or values[0] == 180.0:
                read_g()  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
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
    
def pickup():
    ''' Function to have gripper pick up block by sending 'p' '''
    transmit('P')
    
def dropoff():
    ''' Function to have gripper drop off block by sending 'd' '''
    transmit('D')
    
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
SLEEP_TIME = 0.03
counter = 0

## Inititalize Gyroscope ##
g_ref = read_g() #reference gyrsoscope reading (assuming parallel to a wall)
g_readings = [] #list for gyroscope measurements 
g_readings.append(g_ref) #first reading

## Determine whether left or right wall-following
readings = read_us() #readings in format [Back, Left, Front, Right, BlockSensor]
print(readings)





###############################################
### PART 1: WALL FOLLOWING FOR LOCALIZATION ###
###############################################

OPERATION_LOCALIZE = True

while OPERATION_LOCALIZE == True:
    if readings[3]> readings[1]:  #if RIGHT distance > LEFT distance, follow the LEFT wall
        LZ_R = False # Follow Left Wall
        LZ_L = True
        print(LZ_R, LZ_L)
        print("Following left wall")

    else: 
        LZ_R = True # Follow Right Wall
        LZ_L = False
        print(LZ_R, LZ_L)
        print("Following right wall")

    LZ_L = True
    LZ_R = False   
    ### Previous logic sequence with small steps, run this if the rover has a move forward function that stops after a time delay ###

    while LZ_L: 
        time.sleep(SLEEP_TIME) # delay to not overload with readings
        counter += 1

        #Check US Sensor Readings 
        readings = read_us() #readings in format [Back, Left, Front, Right, BlockSensor]
        print(readings)

        if readings[2] > 13: # 
        move_forward()
        g = read_g()
        print(g)
        #g_readings.append(read_g()) # Store current counter reading
        #g_readings[counter] = Obstacle_Avoidance_Forward(g_ref, g_readings[counter]) #re-orient and correct current orientation if needed
        #g_new = g_readings[counter] #Update current gyroscope position

        elif readings[3] > 13: # If space in front
            g1 = read_g()
            print(g1)
            move_right_big()
            time.sleep(SLEEP_TIME)
            move_right_big()
            time.sleep(SLEEP_TIME)
            time.sleep(5)
            g2 = read_g()
            g2 = read_g()
            print(g2)
            while (g2[0] - g1[0]) < 85:  #If the turn was not a full 90 degree turn, do another small right turn
                print("adjusting right")
                move_right_small()
                time.sleep(SLEEP_TIME)
                time.sleep(1)
                g2 = read_g()
                print(g2)
            while (g2[0] - g1[0]) > 95:  #If the turn was more than 90 degrees, do a small left correction
                print("adjusting left") 
                move_left_small()
                time.sleep(SLEEP_TIME)
                time.sleep(1)
                g2 = read_g()
                print(g2)
                
            #g = read_g()
            #g_readings.append(g) #Store current counter reading
            #g_new, g_ref_new = Obstacle_Avoidance_Right(g_ref, g_readings[counter]) #re-orient and correct current position
            #g_readings[counter] = g_new #Update current gyroscope position
        elif readings[1] > 13:
            g1 = read_g()
            print(g1)
            move_left_big()
            time.sleep(SLEEP_TIME)
            move_left_big()
            time.sleep(5)
            g2 = read_g()
            g2 = read_g()
            print(g2)
            while g2[0] - g[0] > -85: #if the turn was not a full 90 degrees, do another small turn
                print('adjusting left')
                move_left_small()
                time.sleep(SLEEP_TIME)
                time.sleep(1)
                g2 = read_g()
                print(g2)     
                    
            while (g2[0] - g1[0]) < -95:  #If the turn was more than 90 degrees, do a small right correction
                print("adjusting right") 
                move_right_small()
                time.sleep(SLEEP_TIME)
                time.sleep(1)
                g2 = read_g()
                print(g2)
                
            #g = read_g()
            #g_readings.append(g) #Store current counter reading
            #g_new, g_ref_new = Obstacle_Avoidance_Left(g_ref, g_readings[counter]) #re-orient and correct current position
            #g_readings[counter] = g_new #Update current gyroscope position 
            #g_ref = g_ref_new #Update g_ref 

    while LZ_R: 
        time.sleep(SLEEP_TIME) # delay to not overload with readings
        counter += 1

        #Check US Sensor Readings 
        readings = read_us() #readings in format [Back, Left, Front, Right]
        print(readings)     

        if readings[2] > 10:
        transmit('F')

        #g = read_g()
        g_readings.append(read_g) # Store current counter reading
        g_readings[counter] = Obstacle_Avoidance_Forward(g_ref, g_readings[counter]) #re-orient and correct current orientation if needed
        #g_readings[counter] = g_new #Update current gyroscope position      
        
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
    
    
    OPERATION_LOCALIZE = False
    
    
      
        
'''
############################
### PART 2: BLOCK SEARCH ###
############################

OPERATION_BLOCKSEARCH = True

while OPERATION_BLOCKSEARCH == True:
    # Go to loading zone
    # insert code to get to loading zone
        
        
    # Sense block in front
    if readings[4] < 3: # If block is sensed in front of gripper
        transmit('P') # pick up
        time.sleep(SLEEP_TIME)
        transmit('P')

    OPERATION_BLOCKSEARCH = False





##############################
### PART 3: BLOCK DROP-OFF ###
##############################

OPERATION_DROPOFF = True

# Hard-coded paths to each drop-off zone
B1_Path = []
B2_Path = []
B3_Path = []
B4_Path = []

while OPERATION_DROPOFF == True:
    # First get to drop-off location
    At_Dropoff_Loc = False
    while path != (B1 or B2 or B3 or B4):
        path = input(Enter a valid drop-off zone (B1, B2, B3, or B4): )
    
    if path == 'B1':
        for i in B1_Path:
            transmit(B1_Path[i])
        print("B1 drop-off zone reached!")
        At_Dropoff_Loc = True
    
    elif path == 'B2':
        for i in B2_Path:
            transmit(B2_Path[i])
        print("B2 drop-off zone reached!")
        At_Dropoff_Loc = True
    
    elif path == 'B3':
        for i in B3_Path:
            transmit(B3_Path[i])
        print("B3 drop-off zone reached!")
        At_Dropoff_Loc = True
    
    elif path == 'B4':
        for i in B4_Path:
            transmit(B4_Path[i])
        print("B4 drop-off zone reached!")
        At_Dropoff_Loc = True
    
    #might need code to verify
    
    # Drop block
    if At_Dropoff_Loc == True: # If rover has reached drop-off location
        transmit('D') # drop off
        time.sleep(SLEEP_TIME)
        transmit('D')
        print("Block delivered!")
        
        # Victory twirl!
        move_backward()
        move_backward()
        move_right_big()
        move_right_big()
        move_right_big()
        move_right_big()
    
    OPERATION_DROPOFF = False
'''