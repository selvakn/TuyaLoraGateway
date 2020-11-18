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
} TuyaSerialStruct, TuyaLoraStruct;


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

void TuyaSendString(uint8_t id, char data[]) {

  uint16_t len = strlen(data);
  uint16_t payload_len = 4 + len;
  uint8_t payload_buffer[payload_len];
  payload_buffer[0] = id;
  payload_buffer[1] = TUYA_TYPE_STRING;
  payload_buffer[2] = len >> 8;
  payload_buffer[3] = len & 0xFF;

  for (uint16_t i = 0; i < len; i++) {
    payload_buffer[4+i] = data[i];
  }

  TuyaSendResponse(TUYA_CMD_SET_DP, payload_buffer, payload_len);
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

void ProcessByte(uint8_t serial_in_byte, TUYA *tuyaStruct, void (*callback)())
{
  if (serial_in_byte == 0x55) {            // Start TUYA Packet
    tuyaStruct->cmd_status = 1;
    tuyaStruct->buffer[tuyaStruct->byte_counter++] = serial_in_byte;
    tuyaStruct->cmd_checksum += serial_in_byte;
//            Serial.println("0x55");
  }
  else if (tuyaStruct->cmd_status == 1 && serial_in_byte == 0xAA) { // Only packtes with header 0x55AA are valid
    tuyaStruct->cmd_status = 2;

    tuyaStruct->byte_counter = 0;
    tuyaStruct->buffer[tuyaStruct->byte_counter++] = 0x55;
    tuyaStruct->buffer[tuyaStruct->byte_counter++] = 0xAA;
    tuyaStruct->cmd_checksum = 0xFF;
//            Serial.println("cmd_status: 2");
  }
  else if (tuyaStruct->cmd_status == 2) {
    if (tuyaStruct->byte_counter == 5) { // Get length of data
      tuyaStruct->cmd_status = 3;
      tuyaStruct->data_len = serial_in_byte;
//            Serial.println("cmd_status: 3, found length: " + String(tuyaStruct->data_len));
    }

    tuyaStruct->cmd_checksum += serial_in_byte;
    tuyaStruct->buffer[tuyaStruct->byte_counter++] = serial_in_byte;
  }
  else if ((tuyaStruct->cmd_status == 3) && (tuyaStruct->byte_counter == (6 + tuyaStruct->data_len)) && (tuyaStruct->cmd_checksum == serial_in_byte)) { // Compare checksum and process packet
//        Serial.println("checksum statisfied");

    tuyaStruct->buffer[tuyaStruct->byte_counter++] = serial_in_byte;

    uint16_t DataVal = 0;
    uint8_t dpId = 0;
    uint8_t dpDataType = 0;
    char DataStr[13];

//    char hex_char[setCommandPacketLength(tuyaStruct->data_len) * 3];
//    Serial.println(ToHex_P((unsigned char*)tuyaStruct->buffer, setCommandPacketLength(tuyaStruct->data_len), hex_char, sizeof(hex_char)));

    (callback)();

    tuyaStruct->byte_counter = 0;
    tuyaStruct->cmd_status = 0;
    tuyaStruct->cmd_checksum = 0;
    tuyaStruct->data_len = 0;
  }                                                    // read additional packets from TUYA
  else if (tuyaStruct->byte_counter < TUYA_BUFFER_SIZE - 1) { // add char to string if it still fits
//        Serial.println("tuyaStruct.cmd_status" + String(tuyaStruct->cmd_status) + " tuyaStruct.byte_counter: " + String(tuyaStruct->byte_counter) + " tuyaStruct->data_len: " + String(tuyaStruct->data_len) + " serial_in_byte:" + serial_in_byte);
//        Serial.println("reading more buffer");
    tuyaStruct->buffer[tuyaStruct->byte_counter++] = serial_in_byte;
    tuyaStruct->cmd_checksum += serial_in_byte;
  } else {
    Serial.println("resetting");
    tuyaStruct->byte_counter = 0;
    tuyaStruct->cmd_status = 0;
    tuyaStruct->cmd_checksum = 0;
    tuyaStruct->data_len = 0;
  }
}

void onTuyaSerialMessage() {
  if (TUYA_CMD_HEARTBEAT == TuyaSerialStruct.buffer[3]) {
    uint8_t payload_buffer[] = {0x01};
    Serial.println("respond to heartbeat");
    TuyaSendResponse(TUYA_CMD_HEARTBEAT, payload_buffer, 1);
  } else {
    Serial.println("fire lora packet");
    for (int i = 1; i <= 5; i++) {
      LoRa.beginPacket();
      LoRa.write(TuyaSerialStruct.buffer, setCommandPacketLength(TuyaSerialStruct.data_len));
      LoRa.endPacket();
      delay(300);
    }
  }
}

void onTuyaLoraMessage() {
  Serial.println("received tuya lora packet");
  char hex_char[setCommandPacketLength(TuyaLoraStruct.data_len) * 3];
  Serial.println(ToHex_P((unsigned char*)TuyaLoraStruct.buffer, setCommandPacketLength(TuyaLoraStruct.data_len), hex_char, sizeof(hex_char)));
  TuyaSerial.write(TuyaLoraStruct.buffer, setCommandPacketLength(TuyaLoraStruct.data_len));
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;

  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println("Packet Size: " + String(packetSize));

  String str = "RSSI: " + String(LoRa.packetRssi()) + " Snr: " + String(LoRa.packetSnr());
  TuyaSendString(11, str.c_str());
  while (LoRa.available()) {
    ProcessByte(LoRa.read(), &TuyaLoraStruct, onTuyaLoraMessage);
  }
}


void setup() {
  Serial.begin(9600);
  TuyaSerial.begin(9600);
  TuyaSerialStruct.buffer = (char*)(malloc(TUYA_BUFFER_SIZE));
  TuyaLoraStruct.buffer = (char*)(malloc(TUYA_BUFFER_SIZE));

  LoRa.setPins(SS, RST, DI0);

  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (true);
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
  if (TuyaSerial.available()) {
    ProcessByte(TuyaSerial.read(), &TuyaSerialStruct, onTuyaSerialMessage);
  }
  onReceive(LoRa.parsePacket());
}
