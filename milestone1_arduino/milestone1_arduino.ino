#include <iostream>
#include <string> // String editing commands
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(9, 8); // RX | TX of Bluetooth module

// VARS FOR CHECKING CMD INPUTS
cmd = 0; //holds ascii from serial line
String fwd_str = "f:";
String left_str = "l:";
String right_str = "r:";


// Variables to store the number of encoder pulses for each motor
volatile long motA_counts = 0;
volatile long motB_counts = 0;

int fwd_counts = 0;
int turn_counts = 0;

// Speed calculations. Must be within 0~255 due to PWM limits
int motA_speed = 50; // LEFT WHEEL
int motB_speed = 50; // RIGHT WHEEL
float wheel_dia = 3; // inches
float wheel_dist_apart = 8;
float ecpr = 10; encoder counts per 1 full motor rotation

// connect redboard pins to Arduino digital pins
int enA = 6; // motor A controls
int in1A = 10;
int in2A = 11;
int enB = 5; // motor B controls
int in1B = 12;
int in2B = 13;

// ENCODER VARS
#define ENCODERA_A 2  // motor A encoder
#define ENCODERA_B 4
#define ENCODERB_A 6  // motor B encoder
#define ENCODERB_B 8




// FORWARD OR BACKWARD FUNCTION
void MoveForward(int fwd_input, int motA_speed = 50, int motB_speed = 50) {
  // DRIVE FORWARD
  Serial.println("Encoder counts needed: ");
  Serial.print(fwd_input);

  if (fwd_input >= 0 {
  // Forward direction of Motor A
  digitalWrite(in1A, LOW);
    digitalWrite(in2A, HIGH);
    analogWrite(enA, motA_speed);

    // Forward direction of Motor B
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, HIGH);
    analogWrite(enB, motB_speed);
  }

  // DRIVE BACKWARD
  else if fwd_input < 0 {
  // Backward direction of Motor A
  digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
    analogWrite(enA, motA_speed);

    // Backward direction of Motor B
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);
    analogWrite(enB, motB_speed);
  }

  else {
    Serial.println("Invalid distance.");
  }

  // Stop when distance reached, based on encoder
  if abs(motA_counts) >= abs(fwd_input) { // Both motors get same counts
  analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
}


// TURNING FUNCTIONS
void TurnLeft(int turnl_input, int motA_speed = 50, int motB_speed = 50) {
  if (turnl_input >= 0 {
    Serial.println("Encoder counts needed: ");
    Serial.print(turnl_input);

    // Forward direction of Motor A
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
    analogWrite(enA, motA_speed);

    // Forward direction of Motor B
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, HIGH);
    analogWrite(enB, motB_speed);
  }

  else {
    Serial.println("Invalid angle input.");
  }

  // Stop when distance reached, based on encoder
  if abs(motA_counts) >= abs(turnl_input) { // Both motors get same counts
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
}

void TurnRight(int turnr_input, int motA_speed = 50, int motB_speed = 50) {
  if (turnr_input >= 0 {
    Serial.println("Encoder counts needed: ");
    Serial.print(turnr_input);

    // Forward direction of Motor A
    digitalWrite(in1A, LOW);
    digitalWrite(in2A, HIGH);
    analogWrite(enA, motA_speed);

    // Forward direction of Motor B
    digitalWrite(in1B, HIGH);
    digitalWrite(in2B, LOW);
    analogWrite(enB, motB_speed);
  }

  else {
    Serial.println("Invalid angle input.");
  }

  // Stop when distance reached, based on encoder
  if abs(motA_counts) >= abs(turnr_input) { // Both motors get same counts
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
}


void setup()
{
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT); // motor A
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(ENCODERA_A, INPUT);
  pinMode(ENCODERA_B, INPUT);

  pinMode(enB, OUTPUT); // motor B
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
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
    std::string str_cmd = std::to_string(cmd); // Converts cmd input to string


    // FORWARD OR BACKWARD DRIVE
    if (str_cmd.startsWith(fwd_str)) {  // If driving forward...
      str_cmd.erase(0, 2); // Removes first 2 characters
      int fwd_cmd = stoi(str_cmd); // Takes integer distance input
      fwd_counts = fwd_cmd * ecpr / (wheel_dia * pi); // Counts needed

      MoveForward(fwd_counts, motA_speed, motB_speed);
      
      if fwd_counts >= 0 {
        BTSerial.println("Forward: ");
        BTSerial.print(fwd_cmd);
      }
      else if fwd_counts < 0 {
        BTSerial.println("Backward: ");
        BTSerial.print(fwd_cmd);
      }
    }


    // LEFT TURNING DRIVE
    else if (str_cmd.startsWith(left_str)) {  // If turning...
      str_cmd.erase(0, 2); // Removes first 2 characters
      int turn_cmd = stoi(str_cmd); // Takes integer angle input
      float radians_needed = (turn_cmd / 2) * pi / 180; // Calculating travel distance
      float travel_dist = radians_needed * wheel_dist_apart / 2;
      turn_counts = travel_dist * ecpr / (wheel_dia * pi); // Counts needed

      TurnLeft(turn_counts, motA_speed, motB_speed);
      BTSerial.println("Turn Left: ");
      BTSerial.print(turn_cmd);
    }

    // RIGHT TURNING DRIVE
    else if (str_cmd.startsWith(right_str)) {  // If turning...
      str_cmd.erase(0, 2); // Removes first 2 characters
      int turn_cmd = stoi(str_cmd); // Takes integer angle input
      float radians_needed = (turn_cmd / 2) * pi / 180; // Calculating travel distance
      float travel_dist = radians_needed * wheel_dist_apart / 2;
      turn_counts = travel_dist * ecpr / (wheel_dia * pi); // Counts needed

      TurnRight(turn_counts, motA_speed, motB_speed);
      BTSerial.println("Turn Right: ");
      BTSerial.print(turn_cmd);
    }
  }
  

  // Keep reading from Arduino Serial Monitor and send to Software Serial
  if (Serial.available()) {
    CommsSerial.write(Serial.read());
  }
}



// ENCODER EVENT FOR INTERRUPT CALL !!!!! EDIT THIS
void EncoderEvent() {
  // MOTOR A
  if (digitalRead(ENCODERA_A) == HIGH) {
    if (digitalRead(ENCODERA_B) == LOW) {
      motA_counts++;
    }
    else {
      motA_counts--;
    }
  }

  else {
    if (digitalRead(ENCODERA_B) == LOW) {
      motA_counts--;
    }
    else {
      motA_counts++;
    }
  }

  // MOTOR B
  if (digitalRead(ENCODERB_A) == HIGH) {
    if (digitalRead(ENCODERB_B) == LOW) {
      motB_counts++;
    }
    else {
      motB_counts--;
    }
  }

  else {
    if (digitalRead(ENCODERB_B) == LOW) {
      motB_counts--;
    }
    else {
      motB_counts++;
    }
  }

}
