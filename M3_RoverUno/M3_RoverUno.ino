#include <SoftwareSerial.h> // Software Serial for functions
SoftwareSerial mySerial (A4, A5); //RX | TX 

// Connect motor controller pins to Arduino Digital Pins
//Motor A
int enA = 5;
int in1A = 6;
int in2A = 7;
//Motor B
int enB = 9;
int in1B = 10;
int in2B = 11;
//Motor C
int enC = ;
int in1C = ;
int in2C = ;

int motA_speed = 52;
int motB_speed = 42;
int motC_speed = 30;

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

  pinMode(enC, OUTPUT);
  pinMode(in1C, OUTPUT);
  pinMode(in2C, OUTPUT);
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

    } else if (val == 'r'){
      TurnRight();
      delay(100);
      StopMotor();

    } else if (val == 'L') { // drive left
      TurnLeft();
      delay(725);
      StopMotor();
      mySerial.println("left");

    } else if (val == 'l') { // drive left
      TurnLeft();
      delay(100);
      StopMotor();
      mySerial.println("left");

    } else if (val == 'S') { //Stop Motor
      StopMotor();
      mySerial.println("stop");
    
    } else if (val == 'P') { // pick up block
      CloseGrip();
      delay(70);
      StopMotor();
      mySerial.println("Grippers closed");

    } else if (val == 'D') { // drop off block
      OpenGrip();
      delay(70);
      StopMotor();
      mySerial.println("Grippers opened");
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


// GRIPPERS
void CloseGrip() {
  //Motor C
  digitalWrite(in1C, HIGH);
  digitalWrite(in2C, LOW);
  analogWrite(enC, motC_speed);
}

void OpenGrip() {
  //Motor C
  digitalWrite(in1C, LOW);
  digitalWrite(in2C, HIGH);
  analogWrite(enC, motC_speed);
}