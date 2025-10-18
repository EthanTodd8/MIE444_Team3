#include <SoftwareSerial.h>
SoftwareSerial CommsSerial(10, 11); // RX | TX of Comms Arduino

#define ENCODERA_A 2  // motor A encoder
#define ENCODERA_B 4

#define ENCODERB_A 2  // motor B encoder
#define ENCODERB_B 4

// Variables to store the number of encoder pulses for each motor

volatile long motCount = 0;

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
  Serial.println("Drive Arduino active!");

  // Initiate Software Serial
  CommsSerial.begin(9600);
}



void loop()
{
  // READ FROM COMMS ARDUINO
  if (CommsSerial.available()) {  // if Comms Arduino sent something...
    Serial.write(CommsSerial.read());  // write what's sent
  }



  // IF DRIVE COMMAND INPUTTED // EDIT ALL CODE
  if (CommsSerial.available()) {

    val = CommsSerial.read();

    // FORWARD OR BACKWARD DRIVE
    if (val == 'w') {  // !!!! CHANGE THIS to if its a fwd input
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
    else if () { // !!!! EDIT THIS to be if its a turn cmd
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
