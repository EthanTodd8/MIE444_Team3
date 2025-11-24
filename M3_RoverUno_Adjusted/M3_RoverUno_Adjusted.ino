#include <Servo.h>
#include <SoftwareSerial.h> // Software Serial for functions
SoftwareSerial mySerial (A4, A5); //RX | TX 
Servo myservo; // create servo object called myservo

// Connect motor controller pins to Arduino Digital Pins
//Motor A
int enA = 5;
int in1A = 6;
int in2A = 7;
//Motor B
int enB = 9;
int in1B = 10;
int in2B = 11;
//Servo Motor
int pos = 150; // variable to store servo position

int motA_speed = 52;
int motB_speed = 42;

char val = 0;  //holds ascii from serial line


void setup() {
  //Serial.begin(9600);
  myservo.attach(3); 
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
    
    } else if (val == 'f') { // drive forward a bit
      MoveForward();
      delay(100);
      StopMotor();
      mySerial.println(val);

    } else if (val == 'B') { // drive backward
      MoveBackward();
      delay(250);
      StopMotor();
      mySerial.println("backward");

    } else if (val == 'b') { // drive backward a bit
      MoveBackward();
      delay(100);
      StopMotor();
      mySerial.println(val);

    } else if (val == 'R') { // turn right
      TurnRight();
      delay(855);
      StopMotor();
      mySerial.println("right");

    } else if (val == 'r'){ // turn right a bit
      TurnRight();
      delay(100);
      StopMotor();

    } else if (val == 'L') { // turn left
      TurnLeft();
      delay(725);
      StopMotor();
      mySerial.println("left");

    } else if (val == 'l') { // turn left a bit
      TurnLeft();
      delay(100);
      StopMotor();
      mySerial.println("left");

    } else if (val == 'S') { // stop motor
      StopMotor();
      mySerial.println("stop");

    } else if (val == 'i') { // increase motor B speed
      motB_speed += 3;
      mySerial.println("Increased motor speed to");
      mySerial.print(motB_speed);

    } else if (val == 'd') { // decrease motor B speed
      motB_speed -= 3;
      mySerial.println("Decreased motor speed to ");
      mySerial.print(motB_speed);
    
    } else if (val == 'P') { // pick up block
      CloseGrip();
      mySerial.println("Grippers closed");

    } else if (val == 'D') { // drop off block
      OpenGrip();
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
  for (pos = 150; pos <= 180; pos += 1) { 
    myservo.write(pos);               
    delay(15); 
  }
}

void OpenGrip() {
  //for (pos = 180; pos >= 150; pos -= 1) {  
    myservo.write(150); // quickly open to let block slide off              
    //delay(15); 
  }
}