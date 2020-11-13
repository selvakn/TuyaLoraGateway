#include <SPI.h>
#include <LoRa.h>

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    868E6
#define PABOOST true

#define TUYA_CMD_HEARTBEAT     0x00
#define TUYA_CMD_QUERY_PRODUCT 0x01
#define TUYA_CMD_MCU_CONF      0x02
#define TUYA_CMD_WIFI_STATE    0x03
#define TUYA_CMD_WIFI_RESET    0x04
#define TUYA_CMD_WIFI_SELECT   0x05
#define TUYA_CMD_SET_DP        0x06
#define TUYA_CMD_STATE         0x07
#define TUYA_CMD_QUERY_STATE   0x08
#define TUYA_CMD_SET_TIME      0x1C

#define TUYA_TYPE_BOOL         0x01
#define TUYA_TYPE_VALUE        0x02
#define TUYA_TYPE_STRING       0x03
#define TUYA_TYPE_ENUM         0x04

#define TUYA_BUFFER_SIZE       256
#define TuyaSerial Serial1

struct TUYA {
  uint8_t cmd_status = 0;                 // Current status of serial-read
  uint8_t cmd_checksum = 0;               // Checksum of tuya command
  uint16_t data_len = 0;                   // Data lenght of command
  char *buffer = nullptr;                 // Serial receive buffer
  int byte_counter = 0;                   // Index in serial receive buffer
} Tuya;


void TuyaSendResponse(uint8_t cmd, uint8_t payload[] = nullptr, uint16_t payload_len = 0)
{
  uint8_t checksum = (0xFF + cmd + (payload_len >> 8) + (payload_len & 0xFF));
  TuyaSerial.write(0x55);                  // Tuya header 55AA
  TuyaSerial.write(0xAA);
  TuyaSerial.write((uint8_t)0x00);         // version 00
  TuyaSerial.write(cmd);                   // Tuya command
  TuyaSerial.write(payload_len >> 8);      // following data length (Hi)
  TuyaSerial.write(payload_len & 0xFF);    // following data length (Lo)
  for (uint32_t i = 0; i < payload_len; ++i) {
    TuyaSerial.write(payload[i]);
    checksum += payload[i];
  }
  TuyaSerial.write(checksum);
  TuyaSerial.flush();
}

char* ToHex_P(unsigned char * in, size_t insz, char * out, size_t outsz)
{
  unsigned char * pin = in;
  const char * hex = "0123456789ABCDEF";
  char * pout = out;
  for (; pin < in + insz; pout += 3, pin++) {
    pout[0] = hex[(*pin >> 4) & 0xF];
    pout[1] = hex[ *pin     & 0xF];
    pout[2] = ':';
    if (pout + 3 - out > outsz) {
      /* Better to truncate output string than overflow buffer */
      /* it would be still better to either return a status */
      /* or ensure the target buffer is large enough and it never happen */
      break;
    }
  }
  pout[-1] = 0;
  return out;
}

unsigned int setCommandPacketLength(unsigned int dataLen) {
  return 6 + dataLen + 1; // Preamble(2) + version(1) + command byte[6] (1) + lengths'lengh (2) + length(n) + checksum(1)
}

void ProcessByte(uint8_t serial_in_byte)
{
  if (serial_in_byte == 0x55) {            // Start TUYA Packet
    Tuya.cmd_status = 1;
    Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
    Tuya.cmd_checksum += serial_in_byte;
    //    Serial.println("0x55");
  }
  else if (Tuya.cmd_status == 1 && serial_in_byte == 0xAA) { // Only packtes with header 0x55AA are valid
    Tuya.cmd_status = 2;

    Tuya.byte_counter = 0;
    Tuya.buffer[Tuya.byte_counter++] = 0x55;
    Tuya.buffer[Tuya.byte_counter++] = 0xAA;
    Tuya.cmd_checksum = 0xFF;
    //    Serial.println("cmd_status: 2");
  }
  else if (Tuya.cmd_status == 2) {
    if (Tuya.byte_counter == 5) { // Get length of data
      Tuya.cmd_status = 3;
      Tuya.data_len = serial_in_byte;
      //      Serial.println("cmd_status: 3, found length: " + String(Tuya.data_len));
    }

    Tuya.cmd_checksum += serial_in_byte;
    Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
  }
  else if ((Tuya.cmd_status == 3) && (Tuya.byte_counter == (6 + Tuya.data_len)) && (Tuya.cmd_checksum == serial_in_byte)) { // Compare checksum and process packet
    //      Serial.println("checksum statisfied");

    Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;

    uint16_t DataVal = 0;
    uint8_t dpId = 0;
    uint8_t dpDataType = 0;
    char DataStr[13];

    //      char hex_char[setCommandPacketLength(Tuya.data_len)*3];
    //      Serial.println(ToHex_P((unsigned char*)Tuya.buffer, setCommandPacketLength(Tuya.data_len), hex_char, sizeof(hex_char)));

    if (TUYA_CMD_HEARTBEAT == Tuya.buffer[3]) {
      uint8_t payload_buffer[] = {0x01};
      Serial.println("respond to heartbeat");
      TuyaSendResponse(TUYA_CMD_HEARTBEAT, payload_buffer, 1);
    } else {
      Serial.println("fire lora packet");
      LoRa.beginPacket();
      LoRa.write(Tuya.buffer, setCommandPacketLength(Tuya.data_len));
      LoRa.endPacket();
    }

    Tuya.byte_counter = 0;
    Tuya.cmd_status = 0;
    Tuya.cmd_checksum = 0;
    Tuya.data_len = 0;
  }                                                    // read additional packets from TUYA
  else if (Tuya.byte_counter < TUYA_BUFFER_SIZE - 1) { // add char to string if it still fits
    //      Serial.println("Tuya.cmd_status" + String(Tuya.cmd_status) + " Tuya.byte_counter: " + String(Tuya.byte_counter) + " Tuya.data_len: " + String(Tuya.data_len) + " serial_in_byte:" + serial_in_byte);
    //      Serial.println("reading more buffer");
    Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
    Tuya.cmd_checksum += serial_in_byte;
  } else {
    //      Serial.println("resetting");
    Tuya.byte_counter = 0;
    Tuya.cmd_status = 0;
    Tuya.cmd_checksum = 0;
    Tuya.data_len = 0;
  }
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  byte sender = LoRa.read();
  uint8_t dataType = LoRa.read();
  uint8_t dataVal1 = LoRa.read();
  uint8_t dataVal2 = LoRa.read();
  uint16_t dataVal = dataVal1 << 8 | dataVal2;

  String incoming = "";                 // payload of packet

  while (LoRa.available()) {            // can't use readString() in callback, so
    incoming += (char)LoRa.read();      // add bytes one by one
  }
}


void setup() {
  Serial.begin(9600);
  TuyaSerial.begin(9600);
  Tuya.buffer = (char*)(malloc(TUYA_BUFFER_SIZE));

  LoRa.setPins(SS, RST, DI0);

  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (true);
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
  if (TuyaSerial.available()) {
    ProcessByte(TuyaSerial.read());
  }
  onReceive(LoRa.parsePacket());
}
