import socket
import time
from datetime import datetime
import serial

### FUNCTIONS ###
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    clear_serial()
    SER.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.'''

    start_time = time.time()
    response_raw = ''
    while time.time() < start_time + TIMEOUT_SERIAL:
        if SER.in_waiting:
            response_char = SER.read().decode('ascii')
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    print(f'Raw response was: {response_raw}')

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if SER.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing Serial... Dumped: {SER.read(SER.in_waiting)}')

# Packetization and validation functions
def depacketize(data_raw: str):
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid


def read_us():
    '''Asks sensor board to take a reading of the ultrasonic sensors by sending 'u' over serial'''    
    # FRONT
    packet_tx = packetize('u0')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        u0 = float(responses[0][1])
            
    # RIGHT
    packet_tx = packetize('u1')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        u1 = float(responses[0][1])
            
    # LEFT
    packet_tx = packetize('u2')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        u2 = float(responses[0][1])
            
    # BACK
    packet_tx = packetize('u3')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        u3 = float(responses[0][1])
            
    # BLOCKSENSOR
    packet_tx = packetize('u4')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        u4 = float(responses[0][1])

    values = [u3, u2, u0, u1, u4] # [Back, Left, Front, Right, BlockSensor]

    return values

        
def read_g(offset):
    '''Asks sensor board to take gyroscope reading'''
    #values = []

    packet_tx = packetize('g0')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
        values = [float(responses[0][1])]
        
        if values[0] - offset < 0:
            values = [values[0] - offset + 360]
        elif values[0] - offset > 360: 
            values = [values[0] - offset - 360]
        else:
            values = [values[0] - offset]
        return values


def move_forward():
    '''Function to move rover forward by sending 'F' '''
    packet_tx = packetize('w0:2.5')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()

    
def move_fwd_small():
    '''Function to move rover forward a little by sending 'f' '''
    packet_tx = packetize('w0:1')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()

def move_backward():
    '''Function to move rover backward by sending 'B' '''
    packet_tx = packetize('w0:-2')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
    
def move_bwd_small():
    '''Function to move rover backward a little by sending 'b' '''
    packet_tx = packetize('w0:-1')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()

def move_right_big():
    '''Function to move rover right by sending 'R' '''
    packet_tx = packetize('r0:80')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()
            
def move_right_small():
    ''' Function to move rover right by a small increment by sending 'r' '''
    packet_tx = packetize('r0:2')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()

def move_left_big():
    '''Function to move rover left by sending 'L' '''
    packet_tx = packetize('r0:-80')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()

def move_left_small():
    ''' Function to move rover left by a small increment by sending 'l' '''
    packet_tx = packetize('r0:-2')
    if packet_tx:
        transmit(packet_tx)
        [responses, time_rx] = receive()


#def increase_mot_speed():
#    ''' Function to increase motor B speed by sending 'i' '''
#    transmit('i')
    
#def decrease_mot_speed():
#    ''' Function to decrease motor B speed by sending 'd' '''
#    transmit('d')
        
def pickup():
    ''' Function to have gripper pick up block by sending 'p' '''
    print("Picking...")
    
def dropoff():
    ''' Function to have gripper drop off block by sending 'd' '''
    print("Dropping...")
    
def stop():
    '''Function to stop rover motors from moving'''
    transmit(packetize('xx'))
    [responses, time_rx] = receive()



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
           
    ''' 
    # Adjust motor speed to straighten ######## ASSUMING MOTOR B IS ON RIGHT SIDE!!!
    if adjusted_to > 0: # If mainly adjusted right... rover drifts left... right motor too fast
        decrease_mot_speed() 
        print("Right motor too fast. Decreased speed.")
    elif adjusted_to < 0: # If mainly adjusted left... rover drifts right... right motor too slow
        decrease_mot_speed()
        print("Right motor too slow. Increased speed.")
    '''
    
    print("Successfully re-aligned!")
    



def Move_Fwd_Until(desired_dist_from_wall = 10): 
    '''Function to keep moving forward until a desired distance from a wall is met'''
    while readings[2] > desired_dist_from_wall + 10: 
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
    while readings[2] < desired_dist_from_wall + 10 and readings[2] > desired_dist_from_wall: 
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
    while readings[2] < desired_dist_from_wall - 5: 
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
                
  



############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### Serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_SERIAL = 'COM3'    # COM port identification
TIMEOUT_SERIAL = 1      # Serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = True

############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
else:
    SOURCE = 'serial device ' + PORT_SERIAL
try:
    SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
except serial.SerialException:
    print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0





############## Main section for the open loop control algorithm ##############
###################
###   SET-UPS   ###
###################

## Time & Counter Set-Up
SLEEP_TIME = 1
counter = 0

## Inititalize Gyroscope
print("Initializing gyroscope...")
read_g(0) # Give gyroscope 10s to stabilize
time.sleep(2) 
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

            else:
                print("Wall-following done! Now ending...")
                running = False # End this loop
                LZ_L = False # End outer loop
                OPERATION_LOCALIZE = False # End Phase 1!
                OPERATION_BLOCKSEARCH = True # Begin Phase 2!

    
print("Localization completed.")
    
    
      
        

############################
### PART 2: BLOCK SEARCH ###
############################

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
