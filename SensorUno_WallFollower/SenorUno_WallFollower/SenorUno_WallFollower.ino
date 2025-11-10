#include <NewPing.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az; 
int16_t gx, gy, gz;

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

SoftwareSerial BT(9, 10);
SoftwareSerial mySerial(2, 3);  // RX | TX

const int numSamples = 10;

char rover_cmd_array[] = { 'F', 'B', 'L', 'R', 'S' };

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
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
    
  //Serial.begin(9600);
  BT.begin(9600);
  mySerial.begin(9600);

  // initialize gyroscope device
  //Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify gyroscope connection
  //Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}



void loop() {
  //Serial.println("start");
  BT.listen();
  if (BT.available() > 0) {
    char ch = BT.read();
    //Serial.print("read:");
    //Serial.println(ch);

    if (ch == 'u') {

      // Read IR
      //int irValue = analogRead(irPin);

      // Compute averages (in micro sec)
      unsigned long avg0 = getAveragePing(sonar0);
      unsigned long avg1 = getAveragePing(sonar1);
      unsigned long avg2 = getAveragePing(sonar2);
      unsigned long avg3 = getAveragePing(sonar3);
      unsigned long avg4 = getAveragePing(sonar4);

      // Compute distances (in cm)
      float dist0 = (0.5)*(avg0)*(0.034);
      float dist1 = (0.5)*(avg1)*(0.034);
      float dist2 = (0.5)*(avg2)*(0.034);
      float dist3 = (0.5)*(avg3)*(0.034);
      float dist4 = (0.5)*(avg4)*(0.034);

      // Print results
      BT.print(dist0); BT.print(","); //print results on one line
      BT.print(dist1); BT.print(",");
      BT.print(dist2); BT.print(",");
      BT.print(dist3); BT.print(",");
      BT.print(dist4); BT.print(","); //print order [Back, Left, Front, Right]
      // Serial.print("Sensor 0: "); Serial.println(avg0);
      // Serial.print("Sensor 1: "); Serial.println(avg1);
      // Serial.print("Sensor 2: "); Serial.println(avg2);
      // Serial.print("Sensor 3: "); Serial.println(avg3);
      // Serial.print("Sensor 4: "); Serial.println(avg4);
      //Serial.print("IR: "); Serial.println(irValue);

      BT.println(".");
    }

    // GYROSCOPE
    else if (ch == 'g') {
      accelgyro.getRotation(&gx, &gy, &gz);

      #ifdef OUTPUT_READABLE_ACCELGYRO
        //Serial.println(gx); Serial.print("\t");
        //Serial.print("deg/s"); // rotational velocity
      #endif
        //Serial.println(gx); Serial.print("\t");
        //Serial.print("deg/s"); // rotational velocity
      

    // Take avg readings
    //unsigned long sum_r = 0;
    //for (int i = 0; i < numSamples; i++) {
      //sum_r += accelgyro.getRotation(&gx)*0.02; // converting to rotation
      //delay(20);
    //}
    //float avg_r = (sum_r / numSamples);
    
      BT.print(gx);
    }


    else {  
      // Forward rover commands
      for (int i = 0; i < sizeof(rover_cmd_array); i++) {
        if (ch == rover_cmd_array[i]) {
          mySerial.write(ch);
          //Serial.print("Sent to rover: ");
          //Serial.println(ch);
          break;
        }
      }
    }
  }

  // if (mySerial.available()) {
  //   //char received = mySerial.read();
  //   Serial.print("Received from RoverUno: ");
  //   Serial.println(received);
  // }
}
