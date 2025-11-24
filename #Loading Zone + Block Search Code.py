#Loading Zone + Block Search Code
#Arrives at loading zone 
OPERATION_BLOCKSEARCH = True  # Phase 2
#Takes gyroscope reading for reference orientation
g_ref = read_g(g_offset) # Check alignment
while g_ref[0] == 0.0 or g_ref[0] == 180.0:
        g_ref = read_g(g_offset)  #recursively call read_g if the reading is 0 or 180 degrees (erroneous)
        print("Reference orientation: ", g_ref[0])
        
BLOCK_DETECTED = False
Zone_1 = False 


while OPERATION_BLOCKSEARCH: 
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
            if (readings[2] - readings[4]) >= 10:
                BLOCK_DETECTED = True
                print("BLOCK DETECTED")
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
        pickup() #need to define sensor pickup function in arduino
        print('block has been picked')
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
        
    
#Algorithm.
    
#Moves forward one step 
#Rotates +90 degrees, goes back to position, Rotates -90 degrees, step is 30 degrees
#Checks sensor front and down readings to determine block presence
    #If sensor down reading is closer than front reading, by 50% threshold (can adjust during testing)
    #Print Sensor detected, Make block detcteted case = True 
    #else: 
    #Move forward by 1 block (12 inches) 
    #Rotates +90 degrees, goes back to position, Rotates -90 degrees, step is 30 degrees
    #While rotating: Checks sensor front and down readings to determine block presence
        #If sensor down reading is closer than front reading, by 50% threshold (can adjust during testing)
        #Print Sensor detected, Make block detcteted case = True,exit loop
        #else: 
        #Rotate left and move foward 1 12 inch block
        #Rotates +90 degrees, goes back to position, Rotates -90 degrees, step is 30 degrees
        #While rotating: Checks sensor front and down readings to determine block presence
            #If sensor down reading is closer than front reading, by 50% threshold (can adjust during testing)
            #Print Sensor detected, Make block detcteted case = True, exit loop
            #else: 
            #Move forward 1 12 inch block, rotate left
            #Rotates +90 degrees, goes back to position, Rotates -90 degrees, step is 30 degrees 
            #While rotating: Checks sensor front and down readings to determine block presence
                #If sensor down reading is closer than front reading, by 50% threshold (can adjust during testing)
                #Print Sensor detected, Make block detcteted case = True 
                # else: 
                #Rotate left again and move forward 1 12 inch block
                #Continue loop until block detected
                #Return to loading zone
        
#Block Detected Case = False
#Move to the block position until sensor down reading is less than 5 (to be determined during testing)
#Send 'Close gripper; command to arduino 
                                        #Close gripper command turns servo and closes gripper
                                        #Turns until servo reaches 0 (desired position for closed gripper)
#Print ('Block has been obtained')
#Make Exit Loading Zone = True

#Exit Loading Zone = False 
#Turn until gyroscope reading is back to reference orientation +- 5 degrees
#Move forward until hit a wall 
# Is your right side clear? (ultrasonic sensor reading > threshold)
    #If yes, turn right 90 degrees, move forward until hit a wall
        #Is your left and front side blocked? 
            #Turn right 90 degrees, and move forward until you get to sensor zone 5
    #If no, turn 180 degrees, and move forward until you get to sensor zone 5
#If sensor zone 5 is sensed, print ('Exited Loading Zone')
                                        
                                        

 

    