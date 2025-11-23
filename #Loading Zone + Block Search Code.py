#Loading Zone + Block Search Code
#Arrives at loading zone 
#Takes gyroscope reading for reference orientation
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
                                        
                                        

 

    