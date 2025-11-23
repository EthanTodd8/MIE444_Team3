#include <Servo.h>
#include <SoftwareSerial.h> // Software Serial for functions
//SoftwareSerial mySerial (A4, A5); //RX | TX 
Servo myservo; // create servo object called myservo

int pos = 0; // variable to store servo position
char val = 0;  //holds ascii from serial line

int motA_speed = 52;
int motB_speed = 42;


void setup() {
  Serial.begin(9600);
  myservo.attach(9); 
}

void loop() {
  Serial.print("Input something: ");
  if (Serial.available()) { 
    
    val = Serial.read();
    
    if (val == 'P') { // pick up block
      Serial.println("Received P");
      CloseGrip();
      Serial.println("Grippers closed");

    } else if (val == 'D') { // drop off block
      OpenGrip();
      Serial.println("Grippers opened");

    } else if (val == 'i') { // increase motor B speed
      motB_speed += 1;
      Serial.println(motB_speed);
      
    } else if (val == 'd') { // decrease motor B speed
      motB_speed -= 1;
      Serial.println(motB_speed);
    }

  }
}


// GRIPPER FUNCTIONS
void CloseGrip() {
  for (pos = 70; pos <= 90; pos += 1) { 
    myservo.write(pos);               
    delay(15);  // made it much slower for troubleshooting first
  }
}

void OpenGrip() {
  for (pos = 90; pos >= 0; pos -= 1) {  
    myservo.write(pos);               
    delay(15); 
  }
}