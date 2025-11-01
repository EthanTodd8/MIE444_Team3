#include <SoftwareSerial.h> // Software Serial for functions
SoftwareSerial BT(9, 8); //RX | TX 

// Connect motor controller pins to Arduino Digital Pins
//Motor 1
int enA = 6;
int in1A = 5;
int in2A = 4;
//Motor 2
int enB = 9;
int in1B = 7;
int in2B = 8;

int motA_speed = 110;
int motB_speed = 85;

char val = 0;  //holds ascii from serial line

void setup() {
  Serial.begin(9600);
  Serial.println("KISI Rover Uno is alive!");

  pinMode(enA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
}
void loop() {
  if (Serial.available()) { 
    val = Serial.read();

    if (val == 'f') { // drive forward
      MoveForward();
      delay(15);
      StopMotor();
      Serial.println(val);

    } else if (val == 'b') { // drive backward
      MoveBackward();
      delay(15);
      StopMotor();
      Serial.println("backward");

    } else if (val == 'r') { // drive right
      TurnRight();
      delay(15);
      StopMotor();
      Serial.println("right");

    } else if (val == 'l') { // drive left
      TurnLeft();
      delay(15);
      StopMotor();
      Serial.println("left");

    } else if (val == 's') { //Stop Motor
      StopMotor();
      Serial.println("stop");
    }
  }
}

// FUNCTIONS!!!!!!

void StopMotor() {
  //Motor A
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  analogWrite(enA, 0);

  //Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  analogWrite(enB, 0);
}

void MoveForward() {
  //Motor A
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  analogWrite(enA, motA_speed);

  //Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  analogWrite(enB, motB_speed);
}


void MoveBackward() {
  //Motor A
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  analogWrite(enA, motA_speed);

  //Motor B
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  analogWrite(enB, motB_speed);
}

void TurnRight() {
  //Motor A
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  analogWrite(enA, motA_speed);

  //Motor B
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  analogWrite(enB, motB_speed);
}


void TurnLeft() {
  //Motor A
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  analogWrite(enA, motA_speed);

  //Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  analogWrite(enB, motB_speed);
}