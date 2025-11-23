import time
import serial

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM7'    # COM port identification
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
    while case is True:
        ser.write(send.encode('utf-8')) #sends us cmd to Arduino
        time.sleep(0.001) #add delay
        line = ser.readline().strip().decode('ascii')

        if line:
            # Split using ',' as the delimiter
            values_str = line.split(',') # List to store ultrasonic sensor readings into [Back, Left, Front, Right, BlockSensor]
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

    while case is True:
        ser.write(send.encode('utf-8')) #sends us cmd to Arduino
        time.sleep(0.001) #add delay
        line = ser.readline().strip().decode('ascii')

        if line:
            # Split using ',' as the delimiter
            values_str = line.split(',') 
            case = False
            for i in range(len((values_str))-1):
                values.append(float(values_str[i]))
            if values[0] - offset < 0:
                values = [values[0] - offset + 360]
            elif values[0] - offset > 360: 
                values = [values[0] - offset - 360]
            else:
                values = [values[0] - offset]
            return values
        case = True


def move_forward():
    '''Function to move rover forward by sending 'F' '''
    transmit('F')
    
def move_fwd_small():
    '''Function to move rover forward a little by sending 'f' '''
    transmit('f')

def move_backward():
    '''Function to move rover backward by sending 'B' '''
    transmit('B')
    
def move_bwd_small():
    '''Function to move rover backward a little by sending 'b' '''
    transmit('b')

def move_right_big():
    '''Function to move rover right by sending 'R' '''
    transmit('R')

def move_right_small():
    ''' Function to move rover right by a small increment by sending 'r' '''
    transmit('r')

def move_left_big():
    '''Function to move rover left by sending 'L' '''
    transmit('L')

def move_left_small():
    ''' Function to move rover left by a small increment by sending 'l' '''
    transmit('l')

def increase_mot_speed():
    ''' Function to increase motor B speed by sending 'i' '''
    transmit('i')
    
def decrease_mot_speed():
    ''' Function to decrease motor B speed by sending 'd' '''
    transmit('d')
        
def pickup():
    ''' Function to have gripper pick up block by sending 'p' '''
    transmit('P')
    
def dropoff():
    ''' Function to have gripper drop off block by sending 'd' '''
    transmit('D')
    
def stop():
    '''Function to stop rover motors from moving'''
    transmit('S')



def Straighten(g_current, intended_orientation): 
    '''Method: 1) Take diff, 2) Decide what needs to happen, 3) Adjust, 4) Check again and repeat'''
    print("Re-aligning current angle ", g_current, " to ", intended_orientation)
    g_adjusted = [] 
    g_diff = g_current - intended_orientation
    adjusted_to = 0 # checks which direction we mainly adjusted towards; +ve for right, -ve for left
    
    while abs(g_diff) > 3:
        g_adjusted = read_g(g_offset)
        g_diff = g_adjusted[0] - intended_orientation # Check new difference
        
        # Adjust to be in range-- considered too far left or right up until 180 deg on either direction
        if g_diff > 0:
            if g_diff < 180: # Too far right
                move_left_small()
                adjusted_to += -1
                print("Adjusting left")
            elif g_diff > 180: # Too far left
                g_diff = abs(g_diff - 360)
                move_right_small()
                adjusted_to += 1
                print("Adjusting right")
                
        elif g_diff < 0: 
            if g_diff > -180: # Too far left
                g_diff = abs(g_diff)
                move_right_small()
                adjusted_to += 1
                print("Adjusting right")
            elif g_diff < -180: # Too far right
                g_diff += 360  
                move_left_small()
                adjusted_to += -1
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
            
    # Adjust motor speed to straighten ######## ASSUMING MOTOR B IS ON RIGHT SIDE!!!
    if adjusted_to > 0: # If mainly adjusted right... rover drifts left... right motor too fast
        decrease_mot_speed() 
        print("Right motor too fast. Decreased speed.")
    elif adjusted_to < 0: # If mainly adjusted left... rover drifts right... right motor too slow
        decrease_mot_speed()
        print("Right motor too slow. Increased speed.")
    
    print("Successfully re-aligned!")
    



def Move_Fwd_Until(desired_dist_from_wall = 10): 
    '''Function to keep moving forward until a desired distance from a wall is met'''
    while readings[2] > 20: 
        print("Moving forward...")
        move_forward()
        time.sleep(SLEEP_TIME) # Let rover finish moving-- same delay as in drive Arduino
        g = read_g(g_offset) # Check alignment
        while g[0] == 0.0 or g[0] == 180.0:
            g = read_g(g_offset)  #recursively call read_g if reading is 0 or 180 deg (erroneous)
            print("Current angle: ", g[0])
                
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Straighten(g[0], intended_g)
            
            
    # Move a bit forward if close to wall but still space
    while readings[2] < 20 and readings[2] > desired_dist_from_wall: 
        print("Moving forward a little bit...")
        move_fwd_small()
        time.sleep(0.5) # Let rover finish moving-- same delay as in drive Arduino
        g = read_g(g_offset) # Check alignment
        while g[0] == 0.0 or g[0] == 180.0:
            g = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
        print("Current angle: ", g[0])
            
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Straighten(g[0], intended_g)
                
                
    # Move a bit backward if too close to wall
    while readings[2] < 5: 
        print("Moving backward a little bit...")
        move_bwd_small()
        time.sleep(0.5) # Let rover finish moving 
        g = read_g(g_offset) # Check alignment
        while g[0] == 0.0 or g[0] == 180.0:
            g = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
        print("Current angle: ", g[0])
            
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Straighten(g[0], intended_g)
            
    print("Wall encountered!")  
    


def Turn_Right_90():
    '''Function to perfectly turn 90 deg CW'''
    print("Turning right 90 deg...")
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
        g = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
    print("Current angle: ", g[0])
                
    if abs(g[0] - intended_g) > 3: # Straighten as needed
        Straighten(g[0], intended_g)
    
    
                    
def Turn_Left_90():
    '''Function to perfectly turn 90 deg CCW'''
    print("Turning left 90 deg...")
    intended_g += 90 # New intended orientation

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
        g = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
    print("Current angle: ", g[0])
                
    if abs(g[0] - intended_g) > 3: # Straighten as needed
        Straighten(g[0], intended_g)
                
                


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
print("Angle offset: ", g_offset)
print("Gyroscope zeroed successfully.")

readings = read_us() # [Back, Left, Front, Right, BlockSensor]
print(readings)



###############################################
### PART 1: WALL FOLLOWING FOR LOCALIZATION ###
###############################################

OPERATION_LOCALIZE = True      # Phase 1
OPERATION_BLOCKSEARCH = False  # Phase 2
OPERATION_DROPOFF = False      # Phase 3

print("PHASE 1: WALL-FOLLOWING TO LOCALIZE")

while OPERATION_LOCALIZE == True:
    if readings[3]> readings[1]:  #if RIGHT distance > LEFT distance, follow the LEFT wall
        LZ_R = False # Follow Left Wall
        LZ_L = True
        print("Following left wall")

    else: 
        LZ_R = True # Follow Right Wall
        LZ_L = False
        print("Following right wall")

    LZ_L = True
    LZ_R = False
    print("Follow right wall: ", LZ_R, "; Follow left wall: ", LZ_L)   
    ### Previous logic sequence with small steps, run this if the rover has a move forward function that stops after a time delay ###


    while LZ_L: 
        time.sleep(SLEEP_TIME) # delay to not overload with readings
        counter += 1

        #Check US Sensor Readings 
        readings = read_us() #readings in format [Back, Left, Front, Right, BlockSensor]
        print(readings)


        # Move forward if space in front
        if readings[2] > 15: 
            print("Moving forward...")
            move_forward()
            time.sleep(1) # Let rover finish moving-- same delay as in drive Arduino
            g = read_g(g_offset) # Check alignment
            while g[0] == 0.0 or g[0] == 180.0:
                g = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
            print("Current angle: ", g[0])
                
            if abs(g[0] - intended_g) > 3: # Straighten as needed
                Straighten(g[0], intended_g)
            
        
                    
        # Turn right if space on right side
        elif readings[3] > 13: 
            Turn_Right_90()
                    
                
        # Turn left if space on left side  
        elif readings[1] > 13:
            Turn_Left_90()
        
        else:
            print("Ending wall-following...")
            OPERATION_LOCALIZE = False # End Phase 1!
            OPERATION_BLOCKSEARCH = True # Begin Phase 2!

    
    #if : # INSERT CONDITION FOR PROVING SUCCESSFUL LOCALIZATION
print("Localization completed.")
    
    
      
        
'''
############################
### PART 2: BLOCK SEARCH ###
############################

while OPERATION_BLOCKSEARCH == True:
    # Go to loading zone
    # insert code to get to loading zone
        
    # Spin to determine block location
    while readings[4] > 5:
        move_right_small()
    
    # Sense block in front
    if readings[4] < 5: # If block is sensed in front of gripper
    
        print("Block detected nearby!")
        transmit('P') # pick up
        time.sleep(SLEEP_TIME)
        transmit('P')
    
    if readsing
        print("Block collected successfully!")

    OPERATION_BLOCKSEARCH = False
    OPERATION_DROPOFF = True
'''




##############################
### PART 3: BLOCK DROP-OFF ###
##############################

while OPERATION_DROPOFF == True:
    print("###############################/n### PHASE 3: BLOCK DROP-OFF ###/n###############################")

    # Hard-coded paths
    # ASSUMING WE START AT LOADING ZONE EXIT BLOCK FACING RIGHT

    Goto_Zero = True # run path to zero position
    print("Going to zero position...")

    if Goto_Zero == True:
        centered = False
        
        Move_Fwd_Until()
        Turn_Right_90()
        Move_Fwd_Until()
        Turn_Left_90()
        Move_Fwd_Until(26) # Until 2 blocks away (~24'') from wall
        
        # Perfectly center in zero position
        while centered == False:
            ctr_check = read_us() # [Back, Left, Front, Right, BlockSensor]
            print(ctr_check)
            
            # Check if all sides have desired readings
            horiz_diff = ctr_check[0] - ctr_check[2]
            if ctr_check[1] > 12 and ctr_check[3] > 20 and abs(horiz_diff) < 10:
                print("Inside zero position!")
                centered = True
                Goto_Zero = False
            elif horiz_diff > 10: # Too far right
                move_bwd_small()
                if abs(g[0] - intended_g) > 3: # Straighten as needed
                    Straighten(g[0], intended_g)
            elif horiz_diff < -10: # Too far left
                move_fwd_small()
                if abs(g[0] - intended_g) > 3: # Straighten as needed
                    Straighten(g[0], intended_g)
            else: # sides are closed in
                move_bwd_small()
                if abs(g[0] - intended_g) > 3: # Straighten as needed
                    Straighten(g[0], intended_g)
                            
                

    # ASSUMING WE START FROM MIDDLE OF "ZERO POSITION" FACING RIGHT
    At_Dropoff_Loc = False
    path = ''
    while path != ('B1' or 'B2' or 'B3' or 'B4') and Goto_Zero == False:
        path = input("Enter a valid drop-off zone (B1, B2, B3, or B4): ")

    if path == 'B1':
        Turn_Left_90()
        Move_Fwd_Until()
        At_Dropoff_Loc = True
                
    elif path == 'B2':
        Move_Fwd_Until()
        Turn_Left_90()
        Move_Fwd_Until()
        At_Dropoff_Loc = True
        
    elif path == 'B3':
        Move_Fwd_Until()
        Turn_Right_90()
        Move_Fwd_Until()
        At_Dropoff_Loc = True
                
    elif path == 'B4':
        Turn_Right_90()
        Move_Fwd_Until()
        Turn_Right_90()
        Move_Fwd_Until(26) # Until 2 blocks away (~24'') from wall
        Turn_Right_90()
        Move_Fwd_Until()
        At_Dropoff_Loc = True


    # If at drop-off, drop block and verify
    if At_Dropoff_Loc == True:
        print("Drop-off zone ", path, " reached!")  
        reading_before = read_us()  
        dropoff()
        
        # do a lil shimmy to shake block off, in case
        for t in range(5):
            move_left_small()
            move_right_small() 
        
        # Confirm block is released
        move_backward()
        move_backward()
        reading_after = read_us()
        if abs(reading_after[2] - reading_before[2]) > 12:  
            print("Block delivered!")
            
            # Victory twirl!
            move_right_big()
            move_right_big()
            move_right_big()
            move_right_big()
        
            OPERATION_DROPOFF = False
