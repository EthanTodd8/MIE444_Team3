#include <NewPing.h>
#include <SoftwareSerial.h>

#define TRIG_PIN 8
#define ECHO_PIN_0 13
#define ECHO_PIN_1 12
#define ECHO_PIN_2 4
#define ECHO_PIN_3 7
#define ECHO_PIN_4 2
#define MAX_DISTANCE 200

//int irPin = A0;

NewPing sonar0(TRIG_PIN, ECHO_PIN_0, MAX_DISTANCE);
NewPing sonar1(TRIG_PIN, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIG_PIN, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar3(TRIG_PIN, ECHO_PIN_3, MAX_DISTANCE);
NewPing sonar4(TRIG_PIN, ECHO_PIN_4, MAX_DISTANCE);

SoftwareSerial mySerial(10, 11);

const int numSamples = 10;

char rover_cmd_array[] = { 'f', 'b', 'l', 'r', 's' };

// Helper function: Get the average ping for one sonar
unsigned long getAveragePing(NewPing &sonar) {
  unsigned long sum = 0;

  for (int i = 0; i < numSamples; i++) {
    sum += sonar.ping();
    delay(20);
  }

  return sum / numSamples;
}

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char ch = Serial.read();

    if (ch == 'u') {

      // Read IR
      //int irValue = analogRead(irPin);

      // Compute averages
      unsigned long avg0 = getAveragePing(sonar0);
      unsigned long avg1 = getAveragePing(sonar1);
      unsigned long avg2 = getAveragePing(sonar2);
      unsigned long avg3 = getAveragePing(sonar3);
      unsigned long avg4 = getAveragePing(sonar4);

      // Print results
      Serial.println("Averages:");
      Serial.print("Sensor 0: "); Serial.println(avg0);
      Serial.print("Sensor 1: "); Serial.println(avg1);
      Serial.print("Sensor 2: "); Serial.println(avg2);
      Serial.print("Sensor 3: "); Serial.println(avg3);
      Serial.print("Sensor 4: "); Serial.println(avg4);
      //Serial.print("IR: "); Serial.println(irValue);

      Serial.println(".");
    }

    else {  
      // Forward rover commands
      for (int i = 0; i < sizeof(rover_cmd_array); i++) {
        if (ch == rover_cmd_array[i]) {
          mySerial.write(ch);
          Serial.print("Sent to rover: ");
          Serial.println(ch);
          break;
        }
      }
    }
  }

  if (mySerial.available()) {
    char received = mySerial.read();
    Serial.print("Received from RoverUno: ");
    Serial.println(received);
  }
}