#include <SoftwareSerial.h> // Software Serial for functions
SoftwareSerial mySerial (3, 2); //RX | TX 

// Connect motor controller pins to Arduino Digital Pins
//Motor 1
int enA = 5;
int in1A = 6;
int in2A = 7;
//Motor 2
int enB = 9;
int in1B = 10;
int in2B = 11;

int motA_speed = 52;
int motB_speed = 42;

char val = 0;  //holds ascii from serial line

void setup() {
  //Serial.begin(9600);
  mySerial.begin(9600);
  //Serial.println("KISI Rover Uno is alive!");

  pinMode(enA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
}
void loop() {
  if (mySerial.available()) { 
    val = mySerial.read();

    if (val == 'F') { // drive forward
      MoveForward();
      delay(500);
      StopMotor();
      mySerial.println(val);

    } else if (val == 'B') { // drive backward
      MoveBackward();
      delay(250);
      StopMotor();
      mySerial.println("backward");

    } else if (val == 'R') { // drive right
      TurnRight();
      delay(855);
      StopMotor();
      mySerial.println("right");

    } else if (val == 'L') { // drive left
      TurnLeft();
      delay(725);
      StopMotor();
      mySerial.println("left");

    } else if (val == 'S') { //Stop Motor
      StopMotor();
      mySerial.println("stop");
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

void MoveBackward() {
  //Motor A
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  analogWrite(enA, motA_speed);

  //Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  analogWrite(enB, motB_speed);
}


void MoveForward() {
  //Motor A
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  analogWrite(enA, motA_speed);

  //Motor B
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  analogWrite(enB, motB_speed);
}

void TurnLeft() {
  //Motor A
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  analogWrite(enA, motA_speed);

  //Motor B
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  analogWrite(enB, motB_speed);
}


void TurnRight() {
  //Motor A
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  analogWrite(enA, motA_speed);

  //Motor B
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  analogWrite(enB, motB_speed);
}