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

#define D_CMND_TUYA_MCU "TuyaMCU"
#define D_CMND_TUYA_MCU_SEND_STATE "TuyaSend"
#define D_JSON_TUYA_MCU_RECEIVED "TuyaReceived"

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
  uint8_t data_len = 0;                   // Data lenght of command
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

void TuyaSerialInput()
{
  if (TuyaSerial.available()) {
    uint8_t serial_in_byte = TuyaSerial.read();
    if (serial_in_byte == 0x55) {            // Start TUYA Packet
      Tuya.cmd_status = 1;
      Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
      Tuya.cmd_checksum += serial_in_byte;
      Serial.println("start tuya 0x55");
    }
    else if (Tuya.cmd_status == 1 && serial_in_byte == 0xAA) { // Only packtes with header 0x55AA are valid
      Tuya.cmd_status = 2;

      Tuya.byte_counter = 0;
      Tuya.buffer[Tuya.byte_counter++] = 0x55;
      Tuya.buffer[Tuya.byte_counter++] = 0xAA;
      Tuya.cmd_checksum = 0xFF;
    }
    else if (Tuya.cmd_status == 2) {
      if (Tuya.byte_counter == 5) { // Get length of data
        Tuya.cmd_status = 3;
        Tuya.data_len = serial_in_byte;
        Serial.print("found data_len: ");
        Serial.println(serial_in_byte);
      }
      Tuya.cmd_checksum += serial_in_byte;
      Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
    }
    else if ((Tuya.cmd_status == 3) && (Tuya.byte_counter == (6 + Tuya.data_len)) && (Tuya.cmd_checksum == serial_in_byte)) { // Compare checksum and process packet
      Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;

      uint16_t DataVal = 0;
      uint8_t dpId = 0;
      uint8_t dpDataType = 0;
      char DataStr[13];

      uint16_t len = Tuya.buffer[4] << 8 | Tuya.buffer[5];
      char hex_char[(6 + len + 1)*3];

      Serial.println("Data");      
      Serial.println(ToHex_P((unsigned char*)Tuya.buffer, sizeof(hex_char), hex_char, sizeof(hex_char)));      

      if (TUYA_CMD_HEARTBEAT == Tuya.buffer[3]) {
        uint8_t payload_buffer[] = {0x01};
        TuyaSendResponse(TUYA_CMD_HEARTBEAT, payload_buffer, 1);
      }

      if (TUYA_CMD_SET_DP == Tuya.buffer[3]) {
        //55 AA 03 07 00 0D 01 04 00 01 02 02 02 00 04 00 00 00 1A 40
        // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19
        
        uint8_t dpidStart = 6;
        snprintf_P(DataStr, sizeof(DataStr), PSTR("000000000000"));
        while (dpidStart + 4 < Tuya.byte_counter) {
          dpId = Tuya.buffer[dpidStart];
          dpDataType = Tuya.buffer[dpidStart + 1];
          uint16_t dpDataLen = Tuya.buffer[dpidStart + 2] << 8 | Tuya.buffer[dpidStart + 3];
          const unsigned char *dpData = (unsigned char*)&Tuya.buffer[dpidStart + 4];

          if (TUYA_TYPE_BOOL == dpDataType && dpDataLen == 1) {
            DataVal = dpData[0];
          }
          else if (TUYA_TYPE_VALUE == dpDataType && dpDataLen == 4) {
            uint32_t dpValue = (uint32_t)dpData[0] << 24 | (uint32_t)dpData[1] << 16 | (uint32_t)dpData[2] << 8 | (uint32_t)dpData[3] << 0;
            DataVal = dpValue;
          } else if (TUYA_TYPE_STRING == dpDataType) {
          } else if (TUYA_TYPE_ENUM == dpDataType && dpDataLen == 1) {
            DataVal = dpData[0];
          }

          dpidStart += dpDataLen + 4;
        }

        
        Serial.print("dpId: ");
        Serial.println(dpId, HEX);
  
        Serial.print("dpDataType: ");
        Serial.println(dpDataType, HEX);
  
        Serial.print("DataVal: ");
        Serial.println(DataVal);
      
        LoRa.beginPacket();
        LoRa.write(dpId);
        LoRa.write(00);
        LoRa.write(dpDataType);
        LoRa.write(DataVal >> 8);
        LoRa.write(DataVal >> 0);
        LoRa.endPacket();
      }

      Tuya.byte_counter = 0;
      Tuya.cmd_status = 0;
      Tuya.cmd_checksum = 0;
      Tuya.data_len = 0;
    }                                                    // read additional packets from TUYA
    else if (Tuya.byte_counter < TUYA_BUFFER_SIZE - 1) { // add char to string if it still fits
      Tuya.buffer[Tuya.byte_counter++] = serial_in_byte;
      Tuya.cmd_checksum += serial_in_byte;
    } else {
      Tuya.byte_counter = 0;
      Tuya.cmd_status = 0;
      Tuya.cmd_checksum = 0;
      Tuya.data_len = 0;
    }
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

  LoRa.setPins(SS,RST,DI0);

  if (!LoRa.begin(BAND,PABOOST)) {             
    Serial.println("Starting LoRa failed!");
    while (true);
  }

  Serial.println("LoRa init succeeded.");
}

void loop() {
  TuyaSerialInput();
  onReceive(LoRa.parsePacket());
}
