#include <SoftwareSerial.h> // Software Serial for functions
SoftwareSerial BT(9, 8); //RX | TX


// Connect motor controller pins to Arduino Digital Pins
//Motor 1
int enA = 6;
int in1A = 10;
int in2A = 11;
//Motor 2
int enB = 5;
int in1B = 12;
int in2B = 13;

int motA_speed = 110;
int motB_speed = 85;




void setup() {
  // put your setup code here, to run once:
  Serial.println("KISI is alive!");
  Serial.begin(9600); //Initialize Serial Monitor
  BT.begin(9600); //Set baud rate for HC-05 Bluetooth conncection


  //set all motor control pins to outputs, encoder pins to inputs
  pinMode(enA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

}



void loop() {
  // put your main code here, to run repeatedly:
  //BT.println("Loop start");

  
  if (BT.available()) {
    char command = BT.read(); // Input Command using bluetooth connection
    Serial.println(command);

    if (command == 'S') { // STOP!!!
      BT.println("Stop commanded");
      StopMotor();
    }

    else if (command == 'B') { // Backwards
      MoveBackward();
      delay(15);
      StopMotor();
    }

    else if (command == 'L') { // Turn Left
      TurnLeft();
      delay(15);
      StopMotor();
    }

    else if (command == 'R') { // Turn Right
      TurnRight();
      delay(15);
      StopMotor();
    }

    else if (command == 'F') { // Forward
      MoveForward();
      delay(15);
      StopMotor();
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
