#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

RF24 radio(5, 2);               // nRF24L01 (CE,CSN)
RF24Network network(radio);      // Include the radio in the network
const uint16_t this_node = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t node01 = 01;      // Address of the other node in Octal format
const uint16_t node02 = 02;
const uint16_t node03 = 03;

RF24NetworkHeader header1(node01);
RF24NetworkHeader header2(node02);
RF24NetworkHeader header3(node03);

uint8_t recv_data[29];
int i = 0;
int frame_len = 29;

char ack_data[6];

int data_recv_num = 0;
uint8_t serial_data[10] = {0x00};
uint8_t ack2PC[3] = {0xAA, 0xAA, 0xAA};

uint8_t setart_cmd[7] = {0x00, 0x00, 0xAA, 0x00, 0x00, 0x00, 0xAA};

uint32_t timer;

int timeout = 100; // in ms

void setup() {
  SerialUSB.begin(1000000);
  SPI.begin();
  radio.begin();
  network.begin(90, this_node);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
}

void loop() {
  network.update();

  data_recv_num = 0;

  while (SerialUSB.available() > 0) {
    serial_data[data_recv_num] = SerialUSB.read(); // read command from PC
    data_recv_num++;
  }

  if (data_recv_num) {  // if there is command from the PC and all nodes start
    ack2PC[0] = send_cmd(header1, setart_cmd, timeout);
    ack2PC[1] = send_cmd(header2, setart_cmd, timeout);
    ack2PC[2] = send_cmd(header3, setart_cmd, timeout);
    SerialUSB.write(ack2PC, sizeof(ack2PC));
    SerialUSB.println("");
  }

  //===== Receiving =====//
  while ( network.available() ) {     // Is there any incoming data?
    RF24NetworkHeader header;
    network.read(header, &recv_data, sizeof(recv_data)); // Read the incoming data

    SerialUSB.write(recv_data, 29);
  }
}

uint8_t send_cmd(RF24NetworkHeader headerto, uint8_t *cmd, int timeout) {
  uint8_t recv_data[7] = {0x00};

  network.update();
  network.write(headerto, cmd, sizeof(cmd));
  unsigned long started_waiting_at = millis();

  while ( millis()-started_waiting_at < timeout)
  {
    network.update();
    while ( network.available() )
    { // Is there any incoming data?
      RF24NetworkHeader header;
      network.read(header, &recv_data, sizeof(recv_data)); // Read the incoming data
      if (recv_data[2] == 0xAA) return recv_data[2]; // ACK correct
      else return recv_data[2]; // ACK incorrect
    }
  }
  return 0xFF; // timeout
}
