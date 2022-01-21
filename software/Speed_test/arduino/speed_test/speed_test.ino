#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
RF24 radio(2, 10);   // nRF24L01 (CE,CSN)

#define RESTRICT_PITCH

#define  M1_DIR   A2 
#define  M1_PWM   9 
#define  M2_PWM   4 
#define  M2_DIR   A1 
#define ENC1_INT  3  
#define ENC2_INT  7
#define ENC1_LED  8
#define ENC2_LED  6

int frame_len = 29;
uint8_t data2send[29] = {0x01, 0x03, 0x01, 0x04};

RF24Network network(radio);      // Include the radio in the network
const uint16_t this_node = 5;   // Address of our node in Octal format ( 04,031, etc)
const uint16_t master00 = 00;    // Address of the other node in Octal format

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

const int cali_data_len = 30;
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

byte CHK = 0;

uint8_t encoder1Counter = 0;
uint8_t encoder2Counter = 0;

uint8_t cmd[7] = {0x00};
uint8_t ack[7] = {0x00, 0x00, 0xAA, 0x00, 0x00, 0x00, 0xAA};

float shift_reg_x[cali_data_len] = {0};
float shift_reg_y[cali_data_len] = {0};
float shift_reg_z[cali_data_len] = {0};

int cmd_flag  = 0;
int cali_flag = 0;
int cali_done_flag = 0;

int ix = 0;
int iy = 0;
int iz = 0;
int ig = 0;

RF24NetworkHeader header(master00); // (Address where the data is going)

void setup() {
  SerialUSB.begin(1000000);
  pinMode(M1_DIR, OUTPUT); 
  pinMode(M1_PWM, OUTPUT); 
  pinMode(M2_PWM, OUTPUT); 
  pinMode(M2_DIR, OUTPUT);
  pinMode(ENC1_INT, INPUT); 
  pinMode(ENC2_INT, INPUT); 
  pinMode(ENC1_LED, OUTPUT);
  pinMode(ENC2_LED, OUTPUT);

  SerialUSB.begin(115200);
//  SPI.begin();
  radio.begin();
  network.begin(97, this_node);  // (channel, node address)
  radio.setDataRate(RF24_2MBPS);

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

  attachInterrupt(ENC1_INT, ENC1_INT_Routine, FALLING); 
  attachInterrupt(ENC2_INT, ENC2_INT_Routine, FALLING);

  turnOnEncoders();

  timer = micros();
}

void loop() {
  network.update();

  // wait for the command
  while ( network.available() ) {
    network.update();
    RF24NetworkHeader header1;
    network.read(header1, &cmd, sizeof(cmd)); // Read the incoming data
    if (header1.from_node == 0) {// cmd is from master node
      SerialUSB.println("header.from_node");

      if (cmd[2] == 0x00) { // start cmd
        network.update();
//        network.write(header, &ack, sizeof(ack)); // Send the ack

        cmd_flag = 1;
        cali_flag = 0;
      }
      if (cmd[2] != 0x00) { // cali cmd
        network.update();
//        network.write(header, &ack, sizeof(ack)); // Send the ack

        cmd_flag = 0;
        cali_flag = 1;
      }
    }   
  }

//  if(timer4Interrupt==true && cmd_flag == 1) {
  if(timer4Interrupt==true) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    gyroXrate = (g.gyro.x)*57.2958-cali_x;
    gyroYrate = (g.gyro.y)*57.2958-cali_y;
    gyroZrate = (g.gyro.z)*57.2958-cali_z;

    if (cali_flag == 0) {
      for (ix=cali_data_len-1; ix>0; ix--)
        shift_reg_x[ix] = shift_reg_x[ix-1];
      shift_reg_x[0] = gyroXrate;

      for (iy=cali_data_len-1; iy>0; iy--)
        shift_reg_y[iy] = shift_reg_y[iy-1];
      shift_reg_y[0] = gyroYrate;

      for (iz=cali_data_len-1; iz>0; iz--)
        shift_reg_z[iz] = shift_reg_z[iz-1];
      shift_reg_z[0] = gyroZrate;
    }

    accelx = a.acceleration.x;
    accely = a.acceleration.y;
    accelz = a.acceleration.z;

    float dt = (float)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

//    if (ii < cali_data_len) {
    if (cali_flag == 1 && cali_done_flag == 0) { // got the cali cmd and the cali is not finished
      for (ix=0; ix<cali_data_len; ix++) cali_sum_x += shift_reg_x[ix];
      for (iy=0; iy<cali_data_len; iy++) cali_sum_y += shift_reg_y[iy];
      for (iz=0; ix<cali_data_len; iz++) cali_sum_z += shift_reg_z[ix];

      cali_done_flag = 1;
    }

    if (cali_done_flag == 1 && cmd_flag == 1) {
      cali_x = cali_sum_x/cali_data_len;
      cali_y = cali_sum_y/cali_data_len;
      cali_z = cali_sum_z/cali_data_len;

      turnonmotor();

      gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
      gyroYangle += gyroYrate * dt;
      gyroZangle += gyroZrate * dt;

      byte * bx = (byte *) &gyroXangle;
      byte * by = (byte *) &gyroYangle;
      byte * bz = (byte *) &gyroZangle;

      byte * acx = (byte *) &accelx;
      byte * acy = (byte *) &accely;
      byte * acz = (byte *) &accelz;

      data2send[0] = this_node & 0xFF; // Node_address
      data2send[1] = counter;          // Node_address

      memcpy(&data2send[2], bx, 4);    // Gyro data
      memcpy(&data2send[6], by, 4);
      memcpy(&data2send[10], bz, 4);

      memcpy(&data2send[14], acx, 4);    // Accel data
      memcpy(&data2send[18], acy, 4);
      memcpy(&data2send[22], acz, 4);

      data2send[26] = encoder1Counter;  // Encoder_counter_L
      data2send[27] = encoder2Counter;  // Encoder_counter_R

      for (i=0; i<frame_len; i++) {
        CHK += data2send[i];
      }
      data2send[28] = CHK;

      //===== Sending =====//
      network.write(header, &data2send, sizeof(data2send)); // Send the data
    }
    timer4Interrupt=false;
  }
}

//------------HANDLE ENCODER INTERRUPTS---------------
void ENC1_INT_Routine() { 
 encoder1Counter++; 
}

void ENC2_INT_Routine() { 
  encoder2Counter++;
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

void turnOffEncoders() {
  digitalWrite(ENC1_LED, LOW);
  digitalWrite(ENC2_LED, LOW);
}

void turnOnEncoders() {
  digitalWrite(ENC1_LED, HIGH);
  digitalWrite(ENC2_LED, HIGH);
}

void turnonmotor(){
  analogWrite(M1_PWM, 150); // right
  analogWrite(M2_PWM, 150); // left
  digitalWrite(M1_DIR, 0); 
  digitalWrite(M2_DIR, 1);
}

void turnoffmotor(){
  analogWrite(M1_PWM, 0); // right
  analogWrite(M2_PWM, 0); // left
}
