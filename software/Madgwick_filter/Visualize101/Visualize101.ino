#include <MadgwickAHRS.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {
<<<<<<< HEAD
  SerialUSB.begin(9600);
=======
  Serial.begin(115200);
>>>>>>> 05093022cc0abf6f6cd5d3cb8209023aa14c746e

  // start the IMU and filter

  filter.begin(25);

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
<<<<<<< HEAD
    SerialUSB.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  SerialUSB.println("Found LSM9DS1 9DOF");
=======
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");
>>>>>>> 05093022cc0abf6f6cd5d3cb8209023aa14c746e
  // Set the accelerometer range to 2G
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  // Set the gyroscope range to 250 degrees/second
//  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();

  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
//    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
//    ax = convertRawAcceleration(aix);
//    ay = convertRawAcceleration(aiy);
//    az = convertRawAcceleration(aiz);
//    gx = convertRawGyro(gix);
//    gy = convertRawGyro(giy);
//    gz = convertRawGyro(giz);
      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;
      gx = g.gyro.x*57.2958;
      gy = g.gyro.y*57.2958;
      gz = g.gyro.z*57.2958;
//      gx = 0.0;
//      gy = 0.0;
//      gz = 0.0;
      mx = m.magnetic.x;
      my = m.magnetic.y;
      mz = m.magnetic.z;

    // update the filter, which computes orientation
//    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
<<<<<<< HEAD
    //    SerialUSB.print("Orientation: ");
    if (SerialUSB.available()) {
        char input = SerialUSB.read();
        SerialUSB.print(heading);
        SerialUSB.print(",");
        SerialUSB.print(pitch);
        SerialUSB.print(",");
        SerialUSB.println(roll);
=======
    //    Serial.print("Orientation: ");
    if (Serial.available()) {
        char input = Serial.read();
        Serial.print(heading);
        Serial.print(",");
        Serial.print(pitch);
        Serial.print(",");
        Serial.println(roll);
>>>>>>> 05093022cc0abf6f6cd5d3cb8209023aa14c746e
      }
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
