'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic client for sending and receiving data to SimMeR or a robot, for testing purposes
# Some code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

# If using a bluetooth low-energy module (BT 4.0 or higher) such as the HM-10, the ble-serial
# package (https://github.com/Jakeler/ble-serial) is necessary to directly create a serial
# connection between a computer and the device. If using this package, the BAUDRATE constant
# should be left as the default 9600 bps.

import socket
import time
from datetime import datetime
import serial
import numpy as np
import math

# Wrapper functions
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
            chk = 'âœ“'
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

### NEW FUNCTON
'''def calculate_bias(p1_before, p2_before, p1_after, p2_after):
    """
    Calculate the angle of rotation (in radians) between two points 
    before and after a transformation, ignoring translation.

    Parameters
    ----------
    p1_before : tuple (x, y)
        First point before transformation
    p2_before : tuple (x, y)
        Second point before transformation
    p1_after : tuple (x, y)
        First point after transformation
    p2_after : tuple (x, y)
        Second point after transformation

    Returns
    -------
    float
        Rotation angle in radians. Positive = counter-clockwise.
    """

    # Displacement vectors before and after
    v_before = (p2_before[0] - p1_before[0], p2_before[1] - p1_before[1])
    v_after = (p2_after[0] - p1_after[0], p2_after[1] - p1_after[1])

    # Angles of these vectors
    angle_before = math.atan2(v_before[1], v_before[0])
    angle_after = math.atan2(v_after[1], v_after[0])

    # Rotation difference
    rotation = angle_after - angle_before

    # Normalize to [-pi, pi]
    rotation = (rotation + math.pi) % (2 * math.pi) - math.pi

    return rotation'''

############## Main section for the automatic wall follower algorithim ##############
LOOP_PAUSE_TIME = 1 # seconds
# Main loop
AUTOMATIC = True # If true, run this. If false, skip it
counter = 0
sensor_front = []
sensor_left = []
sensor_right = []
sensor_back = []
sensor_down = []
sensor_total = []
CMD_LIST = []

if AUTOMATIC:
    # Pause for a little while so as to not spam commands insanely fast
    time.sleep(LOOP_PAUSE_TIME)

    # Check an ultrasonic sensor 'u0'
    transmit(packetize('u0'))
    [responses, time_rx] = receive()
    print(f"Ultrasonic 0 reading: {response_string('u0',responses)}")
    print(responses)
    sensor_front.append(float(responses[0][1]))
    sensor_total.append(float(responses[0][1]))
    print(sensor_total)

    # Check an ultrasonic sensor 'u1'
    transmit(packetize('u1'))
    [responses,time_rx] = receive()
    print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
    print(responses)
    sensor_right.append(float(responses[0][1]))
    sensor_total.append(float(responses[0][1]))
    print(sensor_total)

    # Check an ultrasonic sensor 'u2'
    transmit(packetize('u2'))
    [responses,time_rx] = receive()
    print(f"Ultrasonic 2 reading: {response_string('u2', responses)}")
    print(responses)
    sensor_left.append(float(responses[0][1]))
    sensor_total.append(float(responses[0][1]))
    print(sensor_total)
    
    # Check an ultrasonic sensor 'u3'
    transmit(packetize('u3'))
    [responses,time_rx] = receive()
    print(f"Ultrasonic 3 reading: {response_string('u3', responses)}")
    print(responses)
    sensor_back.append(float(responses[0][1]))
    sensor_total.append(float(responses[0][1]))
    print(sensor_total)
    
    # Check an ultrasonic sensor 'u4'
    transmit(packetize('u4'))
    [responses,time_rx] = receive()
    print(f"Ultrasonic 4 reading: {response_string('u4', responses)}")
    print(responses)
    sensor_total.append(float(responses[0][1]))
    sensor_down.append(float(responses[0][1]))
    print(sensor_total)
    

    # Determine whether left or right wall-following
    if sensor_right[0] > sensor_left[0]:
        LZ_R = False # Follow Left Wall
        LZ_L = True
        print(LZ_R, LZ_L)
    else: 
        LZ_R = True # Follow Right Wall
        LZ_L = False
        print(LZ_R, LZ_L)

    while LZ_L: 
        counter += 1 # Add 1 to counter for array indexing
        # Check an ultrasonic sensor 'u0'
        transmit(packetize('u0'))
        [responses, time_rx] = receive()
        print(f"Ultrasonic 0 reading: {response_string('u0',responses)}")
        sensor_front.append(float(responses[0][1]))
        sensor_total.append(float(responses[0][1]))
        print(sensor_front[counter])

        # Check an ultrasonic sensor 'u1'
        transmit(packetize('u1'))
        [responses,time_rx] = receive()
        print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
        print(responses)
        sensor_right.append(float(responses[0][1]))
        sensor_total.append(float(responses[0][1]))
        print(sensor_right[counter])

        # Check an ultrasonic sensor 'u2'
        transmit(packetize('u2'))
        [responses,time_rx] = receive()
        print(f"Ultrasonic 2 reading: {response_string('u2', responses)}")
        print(responses)
        sensor_left.append(float(responses[0][1]))
        sensor_total.append(float(responses[0][1]))
        print(sensor_left[counter])

        #Check an ultrasonic sensor 'u3'
        transmit(packetize('u3'))
        [responses,time_rx] = receive()
        sensor_back.append(float(responses[0][1]))
        sensor_total.append(float(responses[0][1]))
        
        
        #Check an ultrasonic sensor 'u4'
        transmit(packetize('u3'))
        [responses,time_rx] = receive()
        sensor_back.append(float(responses[0][1]))
        sensor_total.append(float(responses[0][1]))
        print(sensor_total)
        print(CMD_LIST)
        
        if  sensor_front[counter] > 5:
            packet_tx = packetize('w0:4')
            CMD_LIST.append('F')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()

        elif sensor_right[counter] > 5:
            packet_tx = packetize('r0:90')
            CMD_LIST.append('R')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()

        elif sensor_left[counter] > 5: 
            packet_tx = packetize ('r0:-90')
            CMD_LIST.append('L')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                
        

    while LZ_R: 
        counter += 1 # Add 1 to counter for array indexing
        # Check an ultrasonic sensor 'u0'
        transmit(packetize('u0'))
        [responses, time_rx] = receive()
        print(f"Ultrasonic 0 reading: {response_string('u0',responses)}")
        sensor_total.append(float(responses[0][1]))
        sensor_front.append(float(responses[0][1]))
        print(sensor_front[counter])

        # Check an ultrasonic sensor 'u1'
        transmit(packetize('u1'))
        [responses,time_rx] = receive()
        print(f"Ultrasonic 1 reading: {response_string('u1',responses)}")
        print(responses)
        sensor_total.append(float(responses[0][1]))
        sensor_right.append(float(responses[0][1]))
        print(sensor_right[counter])

        # Check an ultrasonic sensor 'u2'
        transmit(packetize('u2'))
        [responses,time_rx] = receive()
        print(f"Ultrasonic 2 reading: {response_string('u2', responses)}")
        print(responses)
        sensor_total.append(float(responses[0][1]))
        sensor_left.append(float(responses[0][1]))
        print(sensor_left[counter])
        
        #Check an ultrasonic sensor 'u3'
        transmit(packetize('u3'))
        [responses,time_rx] = receive()
        sensor_back.append(float(responses[0][1]))
        sensor_total.append(float(responses[0][1]))
        
        #Check an ultrasonic sensor 'u4'
        transmit(packetize('u4'))
        [responses,time_rx] = receive()
        sensor_down.append(float(responses[0][1]))
        sensor_total.append(float(responses[0][1]))
        
        print(sensor_total)
        print(CMD_LIST)

        if  sensor_front[counter] > 5:
            packet_tx = packetize('w0:4')
            CMD_LIST.append('F')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()

        elif sensor_left[counter] > 5:
            packet_tx = packetize('r0:-90')
            CMD_LIST.append('L')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()

        elif sensor_right[counter] > 5: 
            packet_tx = packetize ('r0:90')
            CMD_LIST.append('R')
            if packet_tx:
                transmit(packet_tx)
                [responses, time_rx] = receive()
                
        


############## Main section for the manual wall follower algorithim ##############

MANUAL = False #Set parameter to TRUE for Manual Control
while MANUAL:
    # Input a command
    cmd = input('Type in a string to send: ')

    # Send the command
    packet_tx = packetize(cmd)
    if packet_tx:
        transmit(packet_tx)

    # Receive the response
    [responses, time_rx] = receive()
    
    if responses[0]:
        print(f"At time '{time_rx}' received from {SOURCE}:\n{response_string(cmd, responses)}")
    else:
        print(f"At time '{time_rx}' received from {SOURCE}:\nMalformed Packet")

    #Sensor Readings
    sensor_cmd = input('Type in Y to read sensor, or N to skip: ')

    if sensor_cmd == 'Y':
        # Check an ultrasonic sensor 'u0'
        packet_tx = packetize('u0')
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            sensor_front = responses[0][1]
        
        #Check an ultrasonic sensor 'u1'
        packet_tx = packetize('u1')
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            sensor_right = responses[0][1]
        
        #Check an ultrasonic sensor 'u2'
        packet_tx = packetize('u2')
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            sensor_left = responses[0][1]

        #Check an ultrasonic sensor 'u3'
        packet_tx = packetize('u3')
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            sensor_back = responses[0][1]

        front_string = f"......{sensor_front}......\n..{sensor_left}..{sensor_right}..\n..........."
        print(front_string)
        
        



'''
    if sensor_left > sensor_right and sensor_left > sensor_front:
        #Send a drive command (turn left)
        transmit(packetize(CMD_LIST[2]))
        [responses,time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1',responses)}")

    elif sensor_right > sensor_left and sensor_right > sensor_front:
        #Send a drive command (turn right)
        transmit(packetize(CMD_LIST[1]))
        [responses,time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1',responses)}")
    
    else:
        #Send a drive forward command
        transmit(packetize(CMD_LIST[0]))
        [responses,time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1',responses)}")

    if sensor_front[counter] < 7:
        if sensor_left[counter] > sensor_right[counter]:
        #Send a drive command (turn left)
            transmit(packetize(CMD_LIST[1]))
            [responses,time_rx] = receive()
            print(f"Drive command response: {response_string('w0:1',responses)}")
            counter+= 1

        elif sensor_right[counter] > sensor_left[counter]:
        #Send a drive command (turn right)
            transmit(packetize(CMD_LIST[2]))
            [responses,time_rx] = receive()
            print(f"Drive command response: {response_string('w0:1',responses)}")
            counter+= 1
    else:
        # Send a drive forward command
        transmit(packetize(CMD_LIST[0]))
        [responses, time_rx] = receive()
        print(f"Drive command response: {response_string('w0:1',responses)}")
        counter+= 1
    if counter > 1:
        angle = calculate_bias(sensor_front[counter-2],sensor_right[counter-2], sensor_front[counter-1], sensor_right[counter-1])
        print(angle)

        if counter > 1:
            #check if right sensor distance is decreasing substantially
            if (sensor_right[counter-1] - sensor_right[counter]) > 1:
                #calculate for the theta needed to orient
                theta_rad = np.arctan(sensor_right[counter-1] - sensor_right[counter])
                theta_deg = np.degrees(theta_rad)
                #Send a drive command (turn left slightly to reorient and avoid collision)
                orient_command = f"{'r0:'}{theta_deg}"
                transmit(packetize(orient_command))
                [responses,time_rx] = receive()
                print(f"Drive command response: {response_string('w0:1',responses)}")
                counter += 1

            elif (sensor_left[counter-1] - sensor_left[counter]) > 1:
                #calculate for the theta needed to orient
                theta_rad = np.arctan(sensor_left[counter-1] - sensor_left[counter])
                theta_deg = np.degrees(theta_rad)
                #Send a drive command (turn right slightly to reorient and avoid collision)
                orient_command = f"{'r0:'}{-theta_deg}"
                transmit(packetize(orient_command))
                [responses,time_rx] = receive()
                print(f"Drive command response: {response_string('w0:1',responses)}")
                counter += 1

        else:
        # Send a drive forward command
            transmit(packetize(CMD_LIST[0]))
            [responses, time_rx] = receive()
            print(f"Drive command response: {response_string('w0:1',responses)}")
            counter+= 1'''