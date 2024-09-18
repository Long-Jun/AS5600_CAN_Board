#include <Wire.h>
#include <AS5600.h>
#include <mcp_can.h>
#include <SPI.h>
#include "Timer.h"

//MCP2515 INT Pin
#define CAN_INT 2

//建立Timer物件
Timer AS5600_CAN_Transmission;
Timer AS5600_Read;
Timer UART_Status_Timer;

//建立MCP2515物件
MCP_CAN CAN(9);

//建立AS5600物件
AMS_5600 ams5600;

int ang, lang = 0;

//MCP_CAN 函式庫 參數

int CAN_ID;
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
int64_t buf[8];
int32_t trans_value_1;
int32_t trans_value_2;

//AS5600上方磁鐵狀態，HIGH=存在；LOW=消失
boolean as5600_magnet_status = LOW;

//AS5600磁鐵角度，DATA範圍 0~4095
int as5600_deg_value = 0;



void setup() {
  Serial.begin(115200);
  Wire.begin();
  SPI.begin();
  pinMode (9, OUTPUT);
  delay(100);

  //啟動MCP2515，CAN-BUS功能
  if (CAN_OK == CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ))
    Serial.println("MCP2515 Connet Successfully!");
  else
    Serial.println("MCP2515 Connet Error.....");

  //設定MCP2515的CAN-BUS模式至標準模式
  CAN.setMode(MCP_NORMAL);

  //啟動AS5600，偵測有無磁鐵
  if (ams5600.detectMagnet() == 0 ) {
    while (1) {
      if (ams5600.detectMagnet() == 1 ) {
        Serial.print("Current Magnitude: ");
        Serial.println(ams5600.getMagnitude());
        break;
      }
      else {
        Serial.println("Can not detect magnet");
      }
      delay(1000);
    }
  }

  AS5600_CAN_Transmission.every(20, as5600_deg_send);
  AS5600_Read.every(10, as5600_deg_read);
  UART_Status_Timer.every(200, print_sensor_value);
}


void loop() {
  AS5600_CAN_Transmission.update();
  AS5600_Read.update();
  UART_Status_Timer.update();
}

void as5600_deg_read() {
  as5600_deg_value = ams5600.getRawAngle();
}

void as5600_deg_send() {
  comm_can_send_as5600_deg(0x70FF, as5600_deg_value);
}

void print_sensor_value() {
  Serial.println(as5600_deg_value);
}




//CAN-BUS 副程式組
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int64(uint8_t* buffer, int64_t number, int32_t *index) {
  buffer[(*index)++] = number >> 56;
  buffer[(*index)++] = number >> 48;
  buffer[(*index)++] = number >> 40;
  buffer[(*index)++] = number >> 32;
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_uint64(uint8_t* buffer, uint64_t number, int32_t *index) {
  buffer[(*index)++] = number >> 56;
  buffer[(*index)++] = number >> 48;
  buffer[(*index)++] = number >> 40;
  buffer[(*index)++] = number >> 32;
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

// CAN commands
typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS
} CAN_PACKET_ID;


void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)duty, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), 1, send_index, buffer);
}

void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)current, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), 1, send_index, buffer);
}

void comm_can_set_current_brake(uint8_t controller_id, float current_brake) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)current_brake, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), 1, send_index, buffer);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), 1, send_index, buffer);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)pos, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), 1, send_index, buffer);
}

void comm_can_send_as5600_deg(uint32_t controller_id, int32_t as5600_value) {
  int32_t send_index = 0;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (uint32_t)as5600_value, &send_index);
  CAN.sendMsgBuf(controller_id, 1, send_index, buffer);
}
