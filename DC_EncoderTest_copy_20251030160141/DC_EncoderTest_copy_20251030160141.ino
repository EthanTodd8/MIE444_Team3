// connect motor controller pins to Arduino digital pins
int enA = 9;
int in1 = 8;
int in2 = 7;
#define ENCODER_A 2
#define ENCODER_B 4

// variables to store the number of encoder pulsesfor each motor
volatile long motCount = 0;

void setup() {
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), EncoderEvent, CHANGE);
  Serial.begin(9600);
}

void loop() {
  // this function will run the motors in both directions at a fixed speed
  // turn on motor in forward direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 200);
  delay(2000);
  // now change motor directions
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(2000);

  Serial.println(motCount);
  delay(500);
}

// encoder event for the interrupt call
void EncoderEvent() {
  if (digitalRead(ENCODER_A) == HIGH) { 
    if (digitalRead(ENCODER_B) == LOW) { //if first signal is high and second signal is low, turning clockwise 
      motCount++;
    } else {
      motCount--;
    }
  } else {
    if (digitalRead(ENCODER_B) == LOW) { //if first signal is low, turning counterclockwise 
      motCount--;
    } else {
      motCount++;
    }
  }
}