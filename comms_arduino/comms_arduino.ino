#include <iostream>
#include <string> // String editing commands
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(9, 10); // RX | TX of Bluetooth module
SoftwareSerial DriveSerial(11, 12); // RX | TX of Drive Arduino

cmd = 0; //holds ascii from serial line
String fwd_str = "w0:";
String turn_str = "r0:";

void setup()
{
  Serial.begin(9600);
  Serial.println("KIZI is alive!");
  Serial.println("Enter command through Bluetooth: ");
  // HC-05 default speed in AT command mode

  BTSerial.begin(38400);
}


void loop()
{
  // Read from Bluetooth module HC-05 and send to Arduino Serial Monitor
  if (BTSerial.available()) {  // If the Bluetooth has something to write...
    Serial.write(BTSerial.read());  // Write what's read from the Bluetooth to serial monitor
  }


  // IDENTIFY AND SEND DRIVE COMMANDS
  cmd == BTSerial.read();
  std::string str_cmd = std::to_string(cmd); // Converts input to string

  if (str_cmd.startsWith(fwd_str)) {  // If driving forward...
    str_cmd.erase(0, 3); // Removes first 3 characters
    int fwd_input = stoi(str_cmd); // Converts to integer
    // insert code to send fwd_input to Drive Arduino
  }

  else if (str_cmd.startsWith(turn_str)) {  // If turning...
    str_cmd.erase(0, 3); // Removes first 3 characters
    int turn_input = stoi(str_cmd); // Converts to integer
    // insert code to send turn_input to Drive Arduino!!!!!!!!!
  }

  else {
    println("Invalid command. No action taken.");
  }



  // Vice versa - Keep reading from Arduino Serial Monitor and send to Software Serial
  if (Serial.available())
    BTSerial.write(Serial.read());
    DriveSerial.write(Serial.read());
}
