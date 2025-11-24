import time
import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve2d

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
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


# LOCALIZATION FUNCTIONS
def sense_u(world, mask, p, SenVal, heading ):
    """
    Models sensor reading update for localization.
    
    Parameters:
        world : np.ndarray
            Ultrasonic map or sensor reading world.
        mask : np.ndarray
            Binary mask of valid regions (1 = free, 0 = blocked).
        p : np.ndarray
            Current probability distribution.
        SenVal : int or float
            Current sensor measurement value.
    Returns:
        pnew : np.ndarray
            Updated and normalized probability map.
    """
    # Probabilities for sensor accuracy
    pHit = 0.9
    pMiss = 0.1
    threshold = 14  # in cm ()
    
    #Claasify into zone type based on sensor value
    u0, u1, u2, u3, u4 = SenVal
    close = [u < threshold for u in [u0, u1, u2, u3]]
    close_count = sum(close)

    if close_count == 0:
        sensor_zone = 0  # free
    elif close_count == 1:
        sensor_zone = 1  #1 side blocked, 3 free
    elif close_count == 2:
        if ((close[0] and close[1]) or (close[0] and close[2]) or (close[2] and close[3]) or (close[1] and close[3])):
            sensor_zone = 2  # adjacent blocking
        elif (close[0] and close[3]) or (close[1] and close[2]):
            sensor_zone = 5  # opposite blocking
    elif close_count == 3:
        sensor_zone = 3  #3 blocked, 1 free
    else:
        sensor_zone = 4  # obstacle (all sides blocked)
    
    print(f"Sensor Readings: {SenVal}, Close Count: {close_count} Classified as: {sensor_zone}") #Debug Mapping Test

    
    # Define expected obstacle patterns per zone type 
    expected = {
        0: np.array([0, 0, 0, 0]),  # free
        1: np.array([1, 0, 0, 0]),  # partial (e.g. wall behind)
        2: np.array([1, 1, 0, 0]),  # adjacent (e.g. corner)
        3: np.array([0, 0, 1, 0]),  # single side (front)
        4: np.array([1, 1, 1, 1]),  # obstacle (ignored)
        5: np.array([1, 0, 1, 0])   # opposite
    }
    
    # Create multiplier matrix
    mult = np.full_like(world, pMiss, dtype=float) #full like creates a new array with the same shape and type as a given array, filled with a specified value
    mult[world == sensor_zone] = pHit

    # Apply sensor update
    pnew = p * mult

    # Apply mask
    pnew[mask ==4] = 0  # set probabilities to zero where mask indicates obstacle

    # Normalize (avoid divide-by-zero)
    total = np.sum(pnew)
    if total > 0:
        pnew /= total
    else:
        pnew = (mask != 4).astype(float)
        pnew = np.ones_like(pnew) / np.size(pnew)
    return pnew

def move(p, mask, heading, Move):
    """
    Models probabilistic movement and heading update.

    Parameters:
        p : np.ndarray
            Current probability map.
        mask : np.ndarray
            Binary mask for valid regions (1 = free, 0 = blocked).
        heading : float
            Current heading angle in degrees.
        Move : str
            Movement command: 'F', 'L', 'B', 'R'.

    Returns:
        pnew : np.ndarray
            Updated probability map.
        heading : float
            Updated heading after rotation.
    """

    # Movement error kernel (same as MATLAB's K)
    K = np.array([
        [0.1, 0.1, 0.1],
        [0.1, 0.8, 0.1],
        [0.1, 0.1, 0.1]
    ])

    # 2D convolution with same size output
    pnew = convolve2d(p, K, mode='same', boundary='fill', fillvalue=0)

    print(f"heading: {heading}")
    
    #rad = np.deg2rad((heading + 90) % 360)
    rad = np.deg2rad(heading)
    col_step = int(np.round(np.cos(rad)))  # +1 -> right, 0, or -1 -> left
    row_step = int(np.round(np.sin(rad)))  # +1 -> down, 0, or -1 -> up

    step_fraction = 0.4  # 5" / 12"

    # Column shift (positive col_step = move right)
    if col_step == 1:
        # move right: shift contents to the right
        shifted = np.hstack([pnew[:, 1:], np.zeros((pnew.shape[0], 1))])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    elif col_step == -1:
        # move left: shift contents to the left
        shifted = np.hstack([np.zeros((pnew.shape[0], 1)), pnew[:, :-1]])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted

    # Row shift (positive row_step = move down on screen)
    if row_step == 1:
        # move down: shift contents downward
        shifted = np.vstack([pnew[1:, :], np.zeros((1, pnew.shape[1]))])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    elif row_step == -1:
        # move up: shift contents upward
        shifted = np.vstack([np.zeros((1, pnew.shape[1])), pnew[:-1, :]])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    
    pnew[mask ==4] = 0  # set probabilities to zero where mask indicates obstacle

    # Normalize
    total = np.sum(pnew)
    if total > 0:
        pnew /= total
    else:
        pnew = (mask != 4).astype(float)
        pnew = np.ones_like(pnew) / np.size(pnew)
        
    print(f"move: {Move} heading: {heading}")
    return pnew, heading



# MORE ADJUSTMENT FUNCTIONS
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
    



def Move_Fwd_Until(desired_dist_from_wall = 5): 
    '''Function to keep moving forward until a desired distance from a wall is met'''
    readings = read_us()
    print(readings)
    
    while readings[2] > desired_dist_from_wall + 5: 
        readings = read_us()
        print(readings)
    
        print("Moving forward...")
        move_forward()
        time.sleep(SLEEP_TIME) # Let rover finish moving-- same delay as in drive Arduino
        g = read_g(g_offset) # Check alignment
        while g[0] == 0.0 or g[0] == 180.0:
            g = read_g(g_offset)  #recursively call read_g if reading is 0 or 180 deg (erroneous)
        print("Current angle: ", g[0])
        print("Intended angle: ", intended_g)
                
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Straighten(g[0], intended_g)
            
        readings = read_us()
            
            
    # Move a bit forward if close to wall but still space
    while readings[2] < desired_dist_from_wall + 5 and readings[2] > desired_dist_from_wall: 
        readings = read_us()
        print(readings)
    
        print("Moving forward a little bit...")
        move_fwd_small()
        time.sleep(0.5) # Let rover finish moving-- same delay as in drive Arduino
        g = read_g(g_offset) # Check alignment
        while g[0] == 0.0 or g[0] == 180.0:
            g = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
        print("Current angle: ", g[0])
            
        if abs(g[0] - intended_g) > 3: # Straighten as needed
            Straighten(g[0], intended_g)
            
        readings = read_us()
        
                
                
    # Move a bit backward if too close to wall
    if readings[2] <= desired_dist_from_wall: 
        while readings[2] < desired_dist_from_wall - 3: 
            readings = read_us()
            print(readings)
        
            print("Moving backward a little bit...")
            move_bwd_small()
            time.sleep(0.5) # Let rover finish moving 
            g = read_g(g_offset) # Check alignment
            while g[0] == 0.0 or g[0] == 180.0:
                g = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
            print("Current angle: ", g[0])
                
            if abs(g[0] - intended_g) > 3: # Straighten as needed
                Straighten(g[0], intended_g)
            
            readings = read_us()
    
        print("Within desired wall distance.")
                
    print("Wall encountered!")  
    


def Turn_Right_90(intended_g):
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
    
    
                    
def Turn_Left_90(intended_g):
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



################################
###   LOCALIZATION SET-UPS   ###
################################

#Initial placement
heading = 180 # 270° is down, 90 is up, 0 is left, 180 is right

# Initialization of the world
dim1, dim2 = 32, 16   # world dimensions (must match ultra)
locationindex = np.arange(1, dim1 * dim2 + 1).reshape(dim2, dim1)
n = locationindex.size
zones_x, zones_y = dim1 // 4, dim2 // 4  # number of 4×4 blocks

# Define what each zone actually is (the “ground truth”)
zone_layout = np.array([
    [2,1,5,2,4,3,4,3],
    [1,2,4,2,5,0,5,1],
    [5,4,3,4,4,5,4,5],
    [2,5,1,5,5,2,4,3]
])

# Define patterns for each zone type
zone_patterns = {
    0: np.full((4, 4), 0),  # free
    1: np.full((4, 4), 1),  # partial blocking
    2: np.full((4, 4), 2),  # adjacent blocking
    3: np.full((4, 4), 3),  # single side blocking
    4: np.full((4, 4), 4),  # total blocking (obstacle)
    5: np.full((4, 4), 5)   # opposite blocking
}
# Build the full-size map
world = np.zeros((dim2, dim1), dtype=int)
for r in range(zones_y):
    for c in range(zones_x):
        ztype = zone_layout[r, c]
        block = zone_patterns[ztype]
        world[r*4:(r+1)*4, c*4:(c+1)*4] = block

# --- VISUALIZE MAP ---
plt.figure(figsize=(10, 5))

#MASK
M = np.ones_like(world, dtype=int)  # 1 = free
M[world == 4] = 4                   # 4 = obstacle

plt.imshow(world, cmap= 'gray', origin='upper', vmin=0, vmax=5)
plt.title("Ground Truth World Map")
plt.axis('off')

for i in range(world.shape[0]):
    for j in range(world.shape[1]):
        if world[i, j] != 4:
            plt.text(j, i, str(world[i, j]), ha='center', va='center', fontsize=6, color='black')

#plt.show() #Doesn't need to show ground truth map at all times


# --- INITIALIZE PROBABILITY ---
p = np.ones((dim2, dim1)) * (1 / n)






###############################################
### PART 1: WALL FOLLOWING FOR LOCALIZATION ###
###############################################

OPERATION_LOCALIZE = True      # Phase 1
OPERATION_BLOCKSEARCH = False  # Phase 2
OPERATION_DROPOFF = False      # Phase 3

print("PHASE 1: WALL-FOLLOWING TO LOCALIZE")

m_m = []  # Movement commands list
m_u = []  # Ultrasonic sensor measurements list

forward_steps = 0
right_steps = 0
left_steps = 0 

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


        running = True
        while running:

            #Check US Sensor Readings 
            readings = read_us() #readings in format [Back, Left, Front, Right]
            m_u.extend([
                    readings[2],
                    readings[3],
                    readings[1],
                    readings[0],
                    readings[4]
                ]) # Rearranging to [Front, Right, Left, Back, Down]
            print(readings)
            print(m_u)

            # Move forward if space in front
            if readings[2] > 20: 
                print("Moving forward...")
                m_m.append('F')
                move_forward()
                print(m_m)
                
                forward_steps += 1
                index = (forward_steps - 1)*5 #5 readings per step
                cmd = m_m[-1:]
                #visualize movement
                if len(m_u) >= index + 5:
                    current_readings = m_u[-5:]
                    
                    p = sense_u(world, M, p, current_readings, heading)
                    
                    # HeatMap Visualization
                    plt.clf()
                    plt.imshow(p, cmap='hot', interpolation='nearest')
                    plt.title(f"Step: {forward_steps}")
                    plt.pause(0.5)

                    p, heading = move(p, M, heading, cmd.upper())
                    
                    plt.show()
                else:
                    print("Take more ultasonic readings")
                
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
                right_steps += 1
                
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
                left_steps += 1
                time.sleep(SLEEP_TIME)
                time.sleep(5)
                g = read_g(g_offset)
                while g[0] == 0.0 or g[0] == 180.0:
                    g = read_g(0)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
                print("Current angle: ", g[0])
                        
                if abs(g[0] - intended_g) > 3: # Straighten as needed
                    Slight_Straighten(g[0], intended_g)

            
            # Move forward if space in front
            if readings[2] > 20: 
                "Moving forward..."
                m_m.append('F')
                move_forward()
                print(m_m)
                
                forward_steps += 1
                index = (forward_steps - 1)*5 #5 readings per step
                cmd = m_m[-1:]
                
                #Visualize movement
                if len(m_u) >= index + 5:
                    current_readings = m_u[-5:]
                    
                    p = sense_u(world, M, p, current_readings, heading)
                    
                    # HeatMap Visualization
                    plt.clf()
                    plt.imshow(p, cmap='hot', interpolation='nearest')
                    plt.title(f"Step: {forward_steps}")
                    plt.pause(0.5)

                    p, heading = move(p, M, heading, cmd.upper())
                    
                    plt.show()
                else:
                    print("Take more ultasonic readings")
                    
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
                right_steps += 1
                time.sleep(SLEEP_TIME)
                move_right_big()
                right_steps += 1
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
                left_steps += 1
                time.sleep(SLEEP_TIME)
                move_left_big()
                left_steps += 1
                time.sleep(SLEEP_TIME)
                time.sleep(5)
                g = read_g(g_offset)
                print("Current angle: ", g[0])
                        
                if abs(g[0] - intended_g) > 3: # Straighten as needed
                    Slight_Straighten(g[0], intended_g)



            # END WALL FOLLOWING ONCE IN ZERO POSITION
            # [Back, Left, Front, Right, BlockSensor]
            
            # If facing right --> left side short
            if readings[0] > 20 and readings[1] > 10 and readings[2] > 20 and readings[3] > 20:
                print("Inside zero position!")
                print("Ending wall-following...")
                LZ_L = False # End this loop
                OPERATION_LOCALIZE = False # End Phase 1!
                OPERATION_BLOCKSEARCH = True # Begin Phase 2!
            
            # If facing left --> right side short
            elif readings[0] > 20 and readings[1] > 20 and readings[2] > 20 and readings[3] > 10:
                print("Inside zero position!")
                print("Ending wall-following...")
                LZ_L = False # End this loop
                OPERATION_LOCALIZE = False # End Phase 1!
                OPERATION_BLOCKSEARCH = True # Begin Phase 2!
            
            # If facing up --> front short
            elif readings[0] > 20 and readings[1] > 20 and readings[2] > 10 and readings[3] > 20:
                print("Inside zero position!")
                print("Ending wall-following...")
                LZ_L = False # End this loop
                OPERATION_LOCALIZE = False # End Phase 1!
                OPERATION_BLOCKSEARCH = True # Begin Phase 2!
                
            # If facing down --> back short
            elif readings[0] > 10 and readings[1] > 20 and readings[2] > 20 and readings[3] > 20:
                print("Inside zero position!")
                print("Ending wall-following...")
                LZ_L = False # End this loop
                OPERATION_LOCALIZE = False # End Phase 1!
                OPERATION_BLOCKSEARCH = True # Begin Phase 2!
    
print("Localization completed.")
    
    
      
        

############################
### PART 2: BLOCK SEARCH ###
############################

print("PHASE 2: BLOCK SEARCH")

# First go to loading zone entry from zero position
# Change orientation to face left-- right side short
while readings[3] < 10 or readings[3] > 20:
    Turn_Left_90(intended_g)
            
    intended_g += -90 # Update intended orientation
    if intended_g >= 360: # Place intended_g within 0-360 range
        intended_g += -360 # If new intended_g is 450, it should actually be 90
    elif intended_g < 0:
        intended_g += 360
    print("New intended orientation: ", intended_g)

print("Now exiting zero position through left side.")

At_Loading_Zone = False

while At_Loading_Zone == False:
    Move_Fwd_Until()
    
    Turn_Right_90(intended_g)
    intended_g += 90 # Update intended orientation
    if intended_g >= 360: # Place intended_g within 0-360 range
        intended_g += -360 # If new intended_g is 450, it should actually be 90
    elif intended_g < 0:
        intended_g += 360
    print("New intended orientation: ", intended_g)
    
    Move_Fwd_Until()
    
    Turn_Left_90(intended_g)   
    intended_g += -90 # Update intended orientation
    if intended_g >= 360: # Place intended_g within 0-360 range
        intended_g += -360 # If new intended_g is 450, it should actually be 90
    elif intended_g < 0:
        intended_g += 360
    print("New intended orientation: ", intended_g)
    
    Move_Fwd_Until(24)
    
    At_Loading_Zone = True
    
    
    
    
#Loading Zone + Block Search Code
#Arrives at loading zone 

#Takes gyroscope reading for reference orientation
g_ref = read_g(g_offset) # Check alignment
while g_ref[0] == 0.0 or g_ref[0] == 180.0:
    g_ref = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
print("Reference orientation: ", g_ref[0])

BLOCK_DETECTED = False
Zone_1 = False 


while OPERATION_BLOCKSEARCH == True:
    time.sleep(SLEEP_TIME) #sleep
    
    #Check US Sensor Readings 
    readings = read_us() #readings in format [Back, Left, Front, Right, BlockSensor]
    print(readings)
    
    if readings[3] >= 20: #if front is free
        move_fwd_small()
        Zone_1 = True
    else:
        print('front is not free')
        break

    while Zone_1 and not BLOCK_DETECTED:
        g_now = read_g(g_offset)
        #turning right
        g_right = g_now[0] + 90 #or 60 depending on gyro reading 
        
        while True:
            move_right_small()                # small incremental turn
            time.sleep(0.2)
            readings = read_us()             # read DURING rotation
            if (readings[2] - readings[4]) >= 10:
                BLOCK_DETECTED = True
                print("BLOCK DETECTED")
                break
            
            g_now = read_g(g_offset)
            if abs (g_now[0]- g_right) <= 5:
                break
            
        g_now = read_g(g_offset)
        #Turning left from right
        g_left= g_now[0] - 180  
        while True:
            move_left_small()                # small incremental turn
            time.sleep(0.2)
            readings = read_us()             # read DURING rotation
            
            #if readings[4] < 5: # If block is sensed in front of gripper
            if (readings[2] - readings[4]) >= 10:
                BLOCK_DETECTED = True
                print("Block detected nearby!")
                break
            
            g_now = read_g(g_offset)
            if abs (g_now[0]- g_left) <= 5:
                break
            
    
if BLOCK_DETECTED:
    while True: 
        readings = read_us() 
        sense_diff = readings[2] - readings[4] #distance between the block and the nearesrt wall in front
        Move_Fwd_Until(sense_diff)#moves forward until the front sensor 'the block difference' away from the wall
        if readings[4] <= 5:
            break
        pickup() 
        print("Block picked successfully!")
        LOADING_ZONE_EXIT = True 
else: 
    print('No block detected')


LOADING_ZONE_EXIT = True
 #turn back to ref orientation
while LOADING_ZONE_EXIT: 
    g_now = read_g(g_offset)
    Straighten(g_now, g_ref)
    Move_Fwd_Until(15) #go to the far left wall
    readings = read_us() #readings in format [Back, Left, Front, Right, BlockSensor]
    if readings[3] >= 15: #is there space on your right? Move to a corner and turn right
        move_right_big()
        Move_Fwd_Until(15)
        move_right_big()
    else:                   #you're already in a corner facing left, turn right
        move_right_big()
        move_right_big()
    readings = read_us #check readings
    
    #Moving out of loading zone
    if readings[3] <= 15: #if nothing is close to right sensor
        move_forward()
        readings = read_us #check readings again
    else:                                   #you're in sensor-block 5 position
        print('Exited Loading Zone')
        LOADING_ZONE_EXIT = False
        OPERATION_BLOCKSEARCH = False
        OPERATION_DROPOFF = True
        




##############################
### PART 3: BLOCK DROP-OFF ###
##############################

while OPERATION_DROPOFF == True:
    print("PHASE 3: BLOCK DROP-OFF")

    # ASSUMING WE START AT LOADING ZONE EXIT BLOCK FACING RIGHT
    Goto_Zero = True # run path to zero position
    print("Going to zero position...")

    if Goto_Zero == True:
        centered = False
        
        Move_Fwd_Until()
        
        Turn_Right_90(intended_g)
        intended_g += 90 # Update intended orientation
        if intended_g >= 360: # Place intended_g within 0-360 range
            intended_g += -360 # If new intended_g is 450, it should actually be 90
        elif intended_g < 0:
            intended_g += 360
        print("New intended orientation: ", intended_g)
        
        Move_Fwd_Until()
        
        Turn_Left_90(intended_g)
        intended_g += -90 # Update intended orientation
        if intended_g >= 360: # Place intended_g within 0-360 range
            intended_g += -360 # If new intended_g is 450, it should actually be 90
        elif intended_g < 0:
            intended_g += 360
        print("New intended orientation: ", intended_g)
                
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
    path = input("Enter a valid drop-off zone (B1, B2, B3, or B4): ")

    if path == 'B1':
        # Step 1: Turn left
        Turn_Left_90(intended_g)
        intended_g += -90 # Update intended orientation
        if intended_g >= 360: # Place within 0-360 deg
            intended_g += -360 
        elif intended_g < 0:
            intended_g += 360
        print("New intended orientation: ", intended_g)
        
        Move_Fwd_Until()
        At_Dropoff_Loc = True
                
    elif path == 'B2':
        Move_Fwd_Until()
        
        # Step 2: Turn left
        Turn_Left_90(intended_g)
        intended_g += -90 # Update intended orientation
        if intended_g >= 360: # Place within 0-360 deg
            intended_g += -360 
        elif intended_g < 0:
            intended_g += 360
        print("New intended orientation: ", intended_g)
        
        Move_Fwd_Until()
        At_Dropoff_Loc = True
        
    elif path == 'B3':
        Move_Fwd_Until()
        
        # Step 2: Turn right
        Turn_Right_90(intended_g)
        intended_g += 90 # Update intended orientation
        if intended_g >= 360: # Place within 0-360 deg
            intended_g += -360 
        elif intended_g < 0:
            intended_g += 360
        print("New intended orientation: ", intended_g)
        
        Move_Fwd_Until()
        At_Dropoff_Loc = True
                
    elif path == 'B4':
        # Step 1: Turn right
        Turn_Right_90(intended_g)
        intended_g += 90 # Update intended orientation
        if intended_g >= 360: # Place within 0-360 deg
            intended_g += -360 
        elif intended_g < 0:
            intended_g += 360
        print("New intended orientation: ", intended_g)
        
        Move_Fwd_Until()
        
        # Step 3: Turn right
        Turn_Right_90(intended_g)
        intended_g += 90 # Update intended orientation
        if intended_g >= 360: # Place within 0-360 deg
            intended_g += -360 
        elif intended_g < 0:
            intended_g += 360
        print("New intended orientation: ", intended_g)
        
        Move_Fwd_Until(26) # Until 2 blocks away (~24'') from wall
        
        # Step 5: Turn right
        Turn_Right_90(intended_g)
        intended_g += 90 # Update intended orientation
        if intended_g >= 360: # Place within 0-360 deg
            intended_g += -360 
        elif intended_g < 0:
            intended_g += 360
        print("New intended orientation: ", intended_g)
        
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
