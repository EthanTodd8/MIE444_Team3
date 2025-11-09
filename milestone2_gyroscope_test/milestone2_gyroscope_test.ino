// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
#include "I2Cdev.h"
#include "MPU6050.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include <Wire.h>
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

int SDA = A4; // used for data transfer through I2C
int SCL = A5; // provides clock pulse for I2C communication
int INT = 2;


void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(INT), InterruptEvent, CHANGE);

  Serial.begin(9600); //Initialize Serial Monitor
}



void loop() {
  // put your main code here, to run repeatedly:


}


void InterruptEvent () {
}
}
