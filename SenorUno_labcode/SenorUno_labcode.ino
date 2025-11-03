#include <NewPing.h>
#include <SoftwareSerial.h>

//Need to define all pins for the 5 sensors here
#define TRIG_PIN 8
#define ECHO_PIN_0 13
#define ECHO_PIN_1 12
#define ECHO_PIN_2 4
#define ECHO_PIN_3 7
#define ECHO_PIN_4 2
#define MAX_DISTANCE 200
int irPin = A0; //select analog input pin 
int irValue = 0; //variable to store sensor value

// sets up the TRIG_PIN and ECHO_PIN
NewPing sonar0(TRIG_PIN, ECHO_PIN_0, MAX_DISTANCE);
NewPing sonar1(TRIG_PIN, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar2(TRIG_PIN, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar3(TRIG_PIN, ECHO_PIN_3, MAX_DISTANCE);
NewPing sonar4(TRIG_PIN, ECHO_PIN_4, MAX_DISTANCE);

SoftwareSerial mySerial(10, 11);  // RX | TX

const int numSamples = 10;
unsigned long samples[numSamples];
unsigned long pingTime0;
unsigned long pingTime1;
unsigned long pingTime2;
unsigned long pingTime3;
unsigned long pingTime4;

char rover_cmd_array[] = { 'f', 'b', 'l', 'r', 's'};
int sum0 = 0;
int sum1 = 0;
int sum2 = 0;
int sum3 = 0;
int sum4 = 0;

void setup() {
  // Start the Serial connection
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    char ch = Serial.read();

    if (ch == 'u') {
      irValue = analogRead(irPin); 
      //Serial.print("IR Sensor = ");
      //Serial.println(irValue); //change these values to 0 and 1 for dark and light depending on threshold
      //append 0 and 1 values to a list and send list back via software serial

      //NEED TO CALCULATE AVERAGE PINGTIME FOR EACH SENSOR AND RETURN THOSE VALUES
      for (int i = 0; i < numSamples; i++) {
        // Send ping, get ping time in microseconds (uS)
        pingTime0 = sonar0.ping();
        delay(15);
        pingTime1 = sonar1.ping();
        delay(15);
        pingTime2 = sonar2.ping();
        delay(15);
        pingTime3 = sonar3.ping();
        delay(15);
        pingTime4 = sonar4.ping();
        delay(10);

        Serial.print("Sensor 0: "); Serial.println(pingTime0);
        Serial.print("Sensor 1: "); Serial.println(pingTime1);
        Serial.print("Sensor 2: "); Serial.println(pingTime2);
        Serial.print("Sensor 3: "); Serial.println(pingTime3);
        Serial.print("Sensor 4: "); Serial.println(pingTime4);
        //samples[i] = (double)pingTime0;
        sum0 += pingTime0;
        sum1 += pingTime1;
        sum2 += pingTime2;
        sum3 += pingTime3;
        sum4 += pingTime4;

        delay(10);

        pingTime0 = sum0 / numSamples; //calculating average pingtime
        pingTime1 = sum1 / numSamples;
        pingTime2 = sum2/numSamples;
        pingTime3 = sum3/numSamples;
        pingTime4 = sum4/numSamples;

        // Print the sensor value over Serial
        //Serial.print(millis());
        //Serial.print(",");
        //Serial.print(pingTime0);
        //Serial.print("\n");

        //Serial.print("Sensor 0: "); Serial.println(pingTime0);
        //Serial.print("Sensor 1: "); Serial.println(pingTime1);
        //Serial.print("Sensor 2: "); Serial.println(pingTime2);
        //Serial.print("Sensor 3: "); Serial.println(pingTime3);
        //Serial.print("Sensor 4: "); Serial.println(pingTime4);

        delay(1);  // delay of 1ms (1000us)
      }
      Serial.println(".");

    } else {  //if command is in rover list - Send via software serial
      for (int i = 0; i < sizeof(rover_cmd_array); i++) {
        if (ch == rover_cmd_array[i]) {
          mySerial.write(ch);
          Serial.print("Sent to rover");
          Serial.println(ch);
          break;  //stop checking once found
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
