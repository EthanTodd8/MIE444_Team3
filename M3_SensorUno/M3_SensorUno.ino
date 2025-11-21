#include <NewPing.h>
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_EULER

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


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
SoftwareSerial mySerial(A1, A0);  // RX | TX

const int numSamples = 5;

char rover_cmd_array[] = { 'F', 'B', 'L', 'R', 'S', 'l', 'r', 'P', 'D' };

// Helper function: Get the average ping for one sonar
unsigned long getAveragePing(NewPing &sonar) {
  unsigned long sum = 0;

  for (int i = 0; i < numSamples; i++) {
    sum += sonar.ping();
    delay(10);
  }

  return sum / numSamples;
}



void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
    
  //Serial.begin(9600);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately
  BT.begin(9600);
  mySerial.begin(9600);
  Serial.begin(9600);

  mpu.initialize(); // initialize gyroscope device

  devStatus = mpu.dmpInitialize(); // load and configure the DMP for gyroscope

  // GYRO OFFSETS
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true); // turn DMP on
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true; // set our DMP Ready flag so the main loop() function knows it's okay to use it
    packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
  } 
}



void loop() {
  //Serial.println("start");
  BT.listen();
  if (BT.available() > 0) {
    char ch = BT.read();
    //Serial.print("read:");
    //Serial.println(ch);

    if (ch == 'u') {

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
      BT.print(dist4); BT.print(","); //print order [Back, Left, Front, Right, BlockSensor]
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

      //mpuInterrupt = false; // reset interrupt flag and get INT_STATUS byte
      mpuIntStatus = mpu.getIntStatus();

      fifoCount = mpu.getFIFOCount(); // get current FIFO count

      // if overflow, reset it
      //if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO(); //reset fifo buffer
        //Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      //}  
      while (!(mpu.getIntStatus() & 0x02));
        // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_EULER
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetEuler(euler, &q);

          // convert to 360 deg range
          if euler[0] * 180/M_PI < 0 {
          BT.print(euler[0] * 180/M_PI + 360); BT.print(","); BT.println(".");
          } else {
          BT.print(euler[0] * 180/M_PI); BT.print(","); BT.println(".");
          }
        #endif
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
  /*
    mySerial.listen();
   if (mySerial.available()) {
    char received = mySerial.read();
    Serial.print("Received from RoverUno: ");
    Serial.println(received);
  }*/
}
