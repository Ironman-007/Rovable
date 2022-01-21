#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

/* IMU Data */
float gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only

float gyroXrate;
float gyroYrate;
float gyroZrate;

float accelx;
float accely;
float accelz;

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

int cali_data_len = 50;
int i = 0;
int ii = 0;

float cali_sum_x = 0.0;
float cali_x = 0.0;

float cali_sum_y = 0.0;
float cali_y = 0.0;

float cali_sum_z = 0.0;
float cali_z = 0.0;

bool timer4Interrupt = false;
byte counter = 0;


void setup() {
  SerialUSB.begin(115200);

  cali_sum_x = 0.0;
  cali_sum_y = 0.0;
  cali_sum_z = 0.0;

  // Try to initialize!
  mpu.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);
  delay(10);

  gyroXangle = 0.0;
  gyroYangle = 0.0;
  gyroZangle = 0.0;
  startTransmitTimer4(10); //freq in Hz
}

void loop() {
  if(timer4Interrupt==true) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    gyroXrate = (g.gyro.x)*57.2958-cali_x;
    gyroYrate = (g.gyro.y)*57.2958-cali_y;
    gyroZrate = (g.gyro.z)*57.2958-cali_z;

    SerialUSB.print(gyroXrate); SerialUSB.print(",");
    SerialUSB.print(gyroYrate); SerialUSB.print(",");
    SerialUSB.println(gyroZrate);

    accelx = a.acceleration.x;
    accely = a.acceleration.y;
    accelz = a.acceleration.z;

    float dt = (float)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    if (ii < cali_data_len) {
      cali_sum_x += gyroXrate;
      cali_sum_y += gyroYrate;
      cali_sum_z += gyroZrate;
      ii++;
    }
    else {
      cali_x = cali_sum_x/cali_data_len;
      cali_y = cali_sum_y/cali_data_len;
      cali_z = cali_sum_z/cali_data_len;
    }
    timer4Interrupt=false;
  }
}

//------------HANDLE TIMER 4 INTERRUPT---------------
void TC4_Handler()   
{
  // Check for overflow (OVF) interrupt
  if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)             
  {
    timer4Interrupt = true;
    counter++;
    REG_TC4_INTFLAG = TC_INTFLAG_OVF;         // Clear the OVF interrupt flag
  }
}

void startTransmitTimer4(int freq) {
  // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_TC4_COUNT16_CC0 = 48000000/(freq*1024) -1;  // Set the TC4 CC0 register as the TOP value in match frequency mode

  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization

  //NVIC_DisableIRQ(TC4_IRQn);
  //NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_OVF;             // Enable TC4 interrupts
  // REG_TC4_INTENCLR = TC_INTENCLR_OVF;          // Disable TC4 interrupts

  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |    // Set prescaler to 1024, 48MHz/1024 = 46.875kHz
                   TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC4 into match frequency (MFRQ) mode 
                   TC_CTRLA_ENABLE;               // Enable TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
}
