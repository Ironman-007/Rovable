#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

RF24 radio(5, 2);               // nRF24L01 (CE,CSN)
RF24Network network(radio);      // Include the radio in the network
const uint16_t this_node = 00;   // Address of this node in Octal format ( 04,031, etc)

uint8_t recv_data[29];
int i = 0;
int frame_len = 29;

char ack_data[6];

uint8_t serial_data[2] = {0x00};
uint8_t ack2PC[3] = {0xAA, 0xAA, 0xAA};

uint8_t setart_cmd[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t cali_cmd[7]   = {0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x11};
uint8_t stop_cmd[7]   = {0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x11};

uint8_t ack[7]   = {0x00};

uint32_t timer;

int timeout = 100; // in ms

uint16_t node_address;

uint8_t robot_speed;

void setup() {
  SerialUSB.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  SPI.begin();
  radio.begin();
  network.begin(97, this_node);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);
}

void loop() {
  network.update();

  int data_recv_num = 0;

  while (SerialUSB.available() > 0) {
    serial_data[data_recv_num] = SerialUSB.read(); // read command from PC
    data_recv_num++;
  }

//  if (data_recv_num == 2) {
//    SerialUSB.print(serial_data[0]); SerialUSB.print(" ");
//    SerialUSB.print(serial_data[1]);
//  }

  if (serial_data[0] == 0xAA) { // START cmd
    node_address = serial_data[1];
    send_start_cmd(node_address);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);

    serial_data[0] = 0x00;
  }

  if (serial_data[0] == 0x11) { // cali cmd
    node_address = serial_data[1];
    robot_speed = serial_data[2];
    send_cali_cmd(node_address);

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

    serial_data[0] = 0x00;
  }

  if (serial_data[0] == 0xFF) { // stop cmd
    node_address = serial_data[1];
    send_stop_cmd(node_address);

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW

    serial_data[0] = 0x00;
  }

  //===== Receiving =====//
  while ( network.available() ) {     // Is there any incoming data?
    RF24NetworkHeader header;
    network.read(header, &recv_data, sizeof(recv_data)); // Read the incoming data
    if (header.from_node == node_address)
    SerialUSB.write(recv_data, 29);
  }
}

void send_start_cmd(uint16_t node_addr) {
  RF24NetworkHeader header(node_addr);
  send_cmd(header, setart_cmd, timeout);
}

void send_cali_cmd(uint16_t node_addr) {
  RF24NetworkHeader header(node_addr);
  cali_cmd[0] = robot_speed;
  send_cmd(header, cali_cmd, timeout);
}

void send_stop_cmd(uint16_t node_addr) {
  RF24NetworkHeader header(node_addr);
  send_cmd(header, stop_cmd, timeout);
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
