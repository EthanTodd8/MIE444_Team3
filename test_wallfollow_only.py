import time
import serial

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM6'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

ser = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)


### FUNCTIONS ###
def transmit(data):
    '''Transmit data over serial connection'''
    ser.write(data.encode('ascii'))

def read_us():
    '''Asks sensor board to take a reading of the ultrasonic sensors by sending 'u' over serial'''
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
        
def read_g(offset):
    '''Asks sensor board to take gyroscope reading'''
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
            if values[0] - offset < 0:
                values = [values[0] - offset + 360]
            elif values[0] - offset > 360: # this might not be needed since offset vals are always positive
                values = [values[0] - offset - 360]
            else:
                values = [values[0] - offset]
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



# Method: 1) Take diff, 2) Decide what needs to happen, 3) Adjust, 4) Check again and repeat
def Slight_Straighten(g_current, intended_orientation): 
    print("Re-aligning current angle ", g_current, " to ", intended_orientation)
    g_adjusted = [] 
    g_diff = g_current - intended_orientation
    
    while abs(g_diff) > 3:
        g_adjusted = read_g(g_offset)
        g_diff = g_adjusted[0] - intended_orientation # Check new difference
        
        # Adjust to be in range-- considered too far left or right up until 180 deg on either direction
        if g_diff > 0:
            if g_diff < 180: # Too far right
                move_left_small()
                print("Adjusting left")
            elif g_diff > 180: # Too far left
                g_diff = abs(g_diff - 360)
                move_right_small()
                print("Adjusting right")
                
        elif g_diff < 0: 
            if g_diff > -180: # Too far left
                g_diff = abs(g_diff)
                move_right_small()
                print("Adjusting right")
            elif g_diff < -180: # Too far right
                g_diff += 360  
                move_left_small()
                print("Adjusting left")
        else:
            print("No change")
                
        time.sleep(1) # Let movement finish before taking new g reading
        g_adjusted = read_g(g_offset)
        new_g_diff = g_adjusted[0] - intended_orientation # Check new difference
        
        # Set new true distance apart        
        if new_g_diff > 180: 
            g_diff = abs(new_g_diff - 360)
        elif new_g_diff < 0: 
            if new_g_diff > -180:
                g_diff = abs(new_g_diff)
            elif new_g_diff < -180:
                g_diff = new_g_diff + 360  
        else:
            g_diff = new_g_diff
        
    print("Successfully aligned!")
    



###################
###   SET-UPS   ###
###################

## Time & Counter Set-Up
SLEEP_TIME = 1
counter = 0

## Inititalize Gyroscope
print("Initializing gyroscope...")
read_g(0) # Give gyroscope 10s to stabilize
time.sleep(10) 
g = read_g(0)
g_offset = g[0] # A constant used to zero out initial position
intended_g = 0 # A changeable constant used to set the way KISI is meant to face; 0, 90, 180, or 270. Starts at "NORTH"
print("Offset: ", g_offset)
print("Gyroscope zeroed successfully.")

readings = read_us() # readings in format [Back, Left, Front, Right, BlockSensor]
print(readings)



###############################################
### PART 1: WALL FOLLOWING FOR LOCALIZATION ###
###############################################
LZ_L = True

while LZ_L: 

    time.sleep(SLEEP_TIME) # delay to not overload with readings
    counter += 1

    #Check US Sensor Readings 
    readings = read_us() #readings in format [Back, Left, Front, Right, BlockSensor]
    print(readings)


<<<<<<< HEAD
time.sleep(SLEEP_TIME) # delay to not overload with readings
counter += 1

#Check US Sensor Readings 
readings = read_us() #readings in format [Back, Left, Front, Right, BlockSensor]
print(readings)

running = True

while running:

    #Check US Sensor Readings 
    readings = read_us() #readings in format [Back, Left, Front, Right]
    print(readings)

    # Move forward if space in front
    if readings[2] > 20: 
        print("Moving forward...")
        move_forward()
        time.sleep(1) # Let rover finish moving-- same delay as in drive Arduino
        g = read_g(g_offset) # Check alignment
        while g[0] == 0.0 or g[0] == 180.0:
            g = read_g(0)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
        print("Current angle: ", g[0])
            
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Slight_Straighten(g[0], intended_g)
            
                
    # Turn right if space on right side
    elif readings[3] > 13: 
        print("Turning right...")
        intended_g += 90 # New intended orientation

        # Place intended_g within 0-360 range
        if intended_g >= 360:
            intended_g += -360 # If new intended_g is 450, it should actually be 90
        elif intended_g < 0:
            intended_g += 360
                
        move_right_big()
        time.sleep(SLEEP_TIME)
        time.sleep(5)
        g = read_g(g_offset)
        while g[0] == 0.0 or g[0] == 180.0:
            g = read_g(0)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
        print("Current angle: ", g[0])
                
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Slight_Straighten(g[0], intended_g)
                
            
    # Turn left if space on left side  
    elif readings[1] > 13:
        print("Turning left...")
        intended_g += -90 # New intended orientation

        # Place intended_g within 0-360 range
        if intended_g >= 360:
            intended_g += -360 # If new intended_g is 450, it should actually be 90
        elif intended_g < 0:
            intended_g += 360
                
        move_left_big()
        time.sleep(SLEEP_TIME)
        time.sleep(5)
        g = read_g(g_offset)
        while g[0] == 0.0 or g[0] == 180.0:
            g = read_g(0)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
        print("Current angle: ", g[0])
                
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Slight_Straighten(g[0], intended_g)

=======
    # Move forward if space in front
    if readings[2] > 20: 
        "Moving forward..."
        move_forward()
        time.sleep(1) # Let rover finish moving-- same delay as in drive Arduino
        g = read_g(g_offset) # Check alignment
        print("Current angle: ", g[0])
            
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Slight_Straighten(g[0], intended_g)
            
                
    # Turn right if space on right side
    elif readings[3] > 20: 
        print("Turning right...")
        intended_g += 90 # New intended orientation

        # Place intended_g within 0-360 range
        if intended_g >= 360:
            intended_g += -360 # If new intended_g is 450, it should actually be 90
        elif intended_g < 0:
            intended_g += 360
                
        move_right_big()
        time.sleep(SLEEP_TIME)
        move_right_big()
        time.sleep(SLEEP_TIME)
        time.sleep(5)
        g = read_g(g_offset)
        print("Current angle: ", g[0])
                
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Slight_Straighten(g[0], intended_g)
                
            
    # Turn left if space on left side  
    elif readings[1] > 20:
        print("Turning left...")
        intended_g += -90 # New intended orientation

        # Place intended_g within 0-360 range
        if intended_g >= 360:
            intended_g += -360 # If new intended_g is 450, it should actually be 90
        elif intended_g < 0:
            intended_g += 360
                
        move_left_big()
        time.sleep(SLEEP_TIME)
        move_left_big()
        time.sleep(SLEEP_TIME)
        time.sleep(5)
        g = read_g(g_offset)
        print("Current angle: ", g[0])
                
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Slight_Straighten(g[0], intended_g)

>>>>>>> adfb602109fd66b0f60a3d7f22e16653e05b87d6
    else:
        print("Wall-following done!")
    