#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor1, sensor2;

#define sensor1_pin 0
#define sensor2_pin 1

void setup()
{
  pinMode(sensor1_pin, OUTPUT);
  pinMode(sensor2_pin, OUTPUT);
 
  // reset both sensors
  digitalWrite(sensor1_pin, LOW);
  digitalWrite(sensor2_pin, LOW);

  SerialUSB.begin(115200);
  Wire.begin();

  // enable sensor1
  digitalWrite(sensor1_pin, HIGH);
  delay(100);

  sensor1.init();
  sensor1.configureDefault();
//  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
//  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(100);
  sensor1.setAddress(0x54);
//  sensor1.stopContinuous();

  digitalWrite(sensor2_pin, HIGH);
  delay(100);

  sensor2.init();
  sensor2.configureDefault();
//  sensor2.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
//  sensor2.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor2.setTimeout(100);
  sensor2.setAddress(0x56);
//  sensor2.stopContinuous();

  // start interleaved continuous mode with period of 100 ms
//  sensor1.startInterleavedContinuous(100);
//  sensor2.startInterleavedContinuous(100);
}

void loop()
{
  SerialUSB.print("\tRange: ");
  SerialUSB.print(sensor1.readRangeSingleMillimeters()); SerialUSB.print(", ");
  SerialUSB.print(sensor2.readRangeSingleMillimeters());

  SerialUSB.println();
  delay(200);
}
