#include <SoftwareSerial.h> // Software Serial for functions

// Connect motor controller pins to Arduino Digital Pins
//Motor 1
int enA = 6; // Placeholder, we can change these to whatever we want
int in1A = 10;
int in2A = 11;
//Motor 2 
int enB = 5;
int in1B = 12; 
int in2B = 13; 

int set_speed_A = 66;
int set_speed_B = 50;

SoftwareSerial BT(9,8); //RX | TX


// ENCODER VARS
#define ENCODERA_A 2  // motor A encoder
#define ENCODERA_B 4

#define ENCODERB_A 3  // motor B encoder
#define ENCODERB_B 7

// Variables to store the number of encoder pulses for each motor
volatile long motA_count = 0;
volatile long motB_count = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.println("Rover Power On!");
  Serial.begin(9600); //Initialize Serial Monitor
  BT.begin(9600); //Set baud rate for HC-05 Bluetooth conncection


  //set all motor control pins to outputs, encoder pins to inputs
  pinMode(enA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  pinMode(ENCODERA_A, INPUT);
  pinMode(ENCODERB_A, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODERA_A), EncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERB_A), EncoderEvent, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  //BT.println("Loop start");
  if (BT.available()) { // Loop runs if BT connected
  BT.println("message received");
    StopMotor(); //Initialize with stationary car
    char command = BT.read(); // Input Command using bluetooth connection
    //StopMotor(); //Initialize with stationary car
    BT.println(command);

    if (command == 's') { //Forward
        BT.println("Stop commanded");
        StopMotor(); //NEED TO WRITE CODE FOR FORWARD COMMAND void forward
        BT.println("Stop");

    } else if (command == 'b') { // Backwards
      MoveBackward();
      BT.println("Moving backward!"); //NEED TO WRITE CODE FOR BACKWARDS COMMAND void backward

    } else if (command == 'l') { // Turn Left
      TurnLeft();
      BT.println("Turning left!");

    } else if (command == 'r') { //Turn Right
      TurnRight();
      BT.println("Turning right!");

    } else if (command == 'f') { //Stop Motor
      MoveForward();
      BT.println("Moving Forward");
    }
  //delay(2000);
  }
}

void MoveForward()
{
// turn on motor in forward direction
//Motor A 
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, set_speed_A);

  //Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, set_speed_B);
  BT.println("Moving");

  delay(50);

  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 0);

  //Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 0);
  BT.println("Moving");
}

void MoveBackward() {
// turn on motor in forward direction
//Motor A
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
// set speed to 200 out of possible range 0~255
  analogWrite(enA, set_speed_A);

//Motor B
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
// set speed to 200 out of possible range 0~255
  analogWrite(enB, set_speed_B);

  //delay(2000);
}

void TurnRight() {
// turn on motor in forward direction
//Motor A 
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
// set speed to 200 out of possible range 0~255
  analogWrite(enA, set_speed_A);

//Motor B
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
// set speed to 200 out of possible range 0~255
  analogWrite(enB, set_speed_B);

  //delay(2000);
}

void TurnLeft(){
// turn on motor in forward direction
//Motor A 
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
// set speed to 200 out of possible range 0~255
  analogWrite(enA, set_speed_A);

//Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
// set speed to 200 out of possible range 0~255
  analogWrite(enB, set_speed_B);

  //delay(2000);
}
void StopMotor() {
// turn on motor in forward direction
//Motor A 
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
// set speed to 200 out of possible range 0~255
  analogWrite(enA, 0);

//Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
// set speed to 200 out of possible range 0~255
  analogWrite(enB, 0);

  //delay(2000);
}

// ENCODER EVENT FOR INTERRUPT CALL
void EncoderEvent() {
  // MOTOR A
  if (digitalRead(ENCODERA_A) == HIGH) {
    if (digitalRead(ENCODERA_B) == LOW) {
      motA_count++;
    }
    else {
      motA_count--;
    }
  }

  else {
    if (digitalRead(ENCODERA_B) == LOW) {
      motA_count--;
    }
    else {
      motA_count++;
    }
  }
  Serial.println(motA_count);

  // MOTOR B
  if (digitalRead(ENCODERB_A) == HIGH) {
    if (digitalRead(ENCODERB_B) == LOW) {
      motB_count++;
    }
    else {
      motB_count--;
    }
  }

  else {
    if (digitalRead(ENCODERB_B) == LOW) {
      motB_count--;
    }
    else {
      motB_count++;
    }
  }
  //BT.println(motB_count);

}    
