

// defines pins numbers (digital inputs)
int stepPinA = 3;
int dirPinA = 2;
int stepPinB = 4;
int dirPinB = 5;

void setup() {
// Sets the two pins as Outputs
pinMode(stepPinA,OUTPUT);
pinMode(dirPinA,OUTPUT);
pinMode(stepPinB,OUTPUT);
pinMode(dirPinB,OUTPUT);
}

void loop() {
digitalWrite(dirPinA,HIGH); // High or low changes the signal of direction
// We need to figure out how many pulses make a full cycle rotation
for(int x = 0; x < 200; x++) {
digitalWrite(stepPinA,HIGH);
digitalWrite(stepPinB, HIGH);
delayMicroseconds(500);
digitalWrite(stepPinA,LOW);
digitalWrite(stepPinB, LOW);
delayMicroseconds(500);
}
delay(1000); // One second delay
}