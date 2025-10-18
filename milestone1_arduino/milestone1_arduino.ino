#include <iostream>
#include <string> // String editing commands
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(9, 10); // RX | TX of Bluetooth module

// VARS FOR CHECKING CMD INPUTS
cmd = 0; //holds ascii from serial line
String fwd_str = "w0:";
String turn_str = "r0:";


// ENCODER VARS
#define ENCODERA_A 2  // motor A encoder
#define ENCODERA_B 4

#define ENCODERB_A 6  // motor B encoder
#define ENCODERB_B 8

// Variables to store the number of encoder pulses for each motor
volatile long motA_count = 0;
volatile long motB_count = 0;


char val = 0; //holds ascii from serial line
int fwd_duration = 0;
int turn_duration = 0;

// Speed calculations. Must be within 0~255 due to PWM limits
int fwd_speed = 200; // set fwd speed to something within possible range 0~255
int turn_speed = 200; // set turn speed to something within possible range 0~255
float wheel_dia = 1.5; // inches


// connect redboard pins to Arduino digital pins
int enA = 9; // motor A controls
int in1 = 8;
int in2 = 7;

int enB = 4; // motor B controls
int in3 = 3;
int in4 = 2;




void setup()
{
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT); // motor A
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ENCODERA_A, INPUT);
  pinMode(ENCODERA_B, INPUT);

  pinMode(enB, OUTPUT); // motor B
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENCODERB_A, INPUT);
  pinMode(ENCODERB_B, INPUT);

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODERA_A), EncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERB_A), EncoderEvent, CHANGE);

  Serial.begin(9600);
  Serial.println("KIZI is alive!");
  Serial.println("Enter command through Bluetooth: ");
  // HC-05 default speed in AT command mode

  BTSerial.begin(38400);
}




void loop()
{
  // READ FROM BLUETOOTH
  if (BTSerial.available()) {  // If the Bluetooth has something to write...
    Serial.write(BTSerial.read());  // Write what's read from the Bluetooth to serial monitor


      // IDENTIFY AND SEND DRIVE COMMANDS
      cmd = BTSerial.read();
      std::string str_cmd = std::to_string(cmd); // Takes cmd input and converts it to string


      // FORWARD OR BACKWARD DRIVE
      if (str_cmd.startsWith(fwd_str)) {  // If driving forward...
        str_cmd.erase(0, 3); // Removes first 3 characters
        int fwd_input = stoi(str_cmd); // Takes integer distance input
        
        // DRIVE FORWARD OR BACKWARD
        if fwd_input > 0 {
        Serial.println("Forward: ");
          Serial.print(fwd_input);

          // Forward direction of Motor A
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          analogWrite(enA, fwd_speed);

          //fwd_duration = fwd_input*(pi*wheel_dia)INSERT CODE HERE!!!!!!!!!!! // Delay depends on speed of motor and duration based on cmd
          delay(fwd_duration);

          // Forward direction of Motor B
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          analogWrite(enB, fwd_speed);
          delay(fwd_duration);
        }

        else if fwd_input < 0 {
        Serial.println("Backward: ");
          Serial.print(fwd_input);

          // Backward direction of Motor A
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          analogWrite(enA, fwd_speed);

          //fwd_duration = INSERT CODE HERE!!!!!!!!!!! // Delay depends on speed of motor and duration based on cmd
          delay(fwd_duration);

          // Backward direction of Motor B
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          analogWrite(enB, fwd_speed);
          delay(fwd_duration);
        }

      }


      // TURNING DRIVE
      else if (str_cmd.startsWith(turn_str)) {  // If turning...
        str_cmd.erase(0, 3); // Removes first 3 characters
        int turn_input = stoi(str_cmd); // Takes integer distance input
        
        if turn_input > 0 {
        Serial.println("Turn left: ");
          Serial.print(turn_input);

          // Forward direction of Motor A
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          analogWrite(enA, turn_speed);

          //turn_duration = turn_input*(pi*d)INSERT CODE HERE!!!!!!!!!!! // Delay depends on speed of motor and duration based on cmd
          delay(turn_duration);

          // Backward direction of Motor B
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          analogWrite(enB, turn_speed);
          delay(turn_duration);
        }

        if turn_input < 0 {
        Serial.println("Turn right: ");
          Serial.print(turn_input);

          // Forward direction of Motor A
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          analogWrite(enA, turn_speed);

          //turn_duration = turn_input*(pi*d)INSERT CODE HERE!!!!!!!!!!! // Delay depends on speed of motor and duration based on cmd
          delay(turn_duration);

          // Backward direction of Motor B
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          analogWrite(enB, turn_speed);
          delay(turn_duration);
        }
      }

    }

    // Keep reading from Arduino Serial Monitor and send to Software Serial
    if (Serial.available()) {
      CommsSerial.write(Serial.read());
    }
  }
