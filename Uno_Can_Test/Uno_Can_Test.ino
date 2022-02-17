/**
 * George Spearing
 * February 2022
 * Telemetry Script. Read CAN into variables
 * Then LoRa can pick and send those variables
 * at a different rate
 */


#include <Arduino.h>
#include <mcp_can.h>

// CAN Settings
#define PIN_SPI_CAN_CS 5 // CAN SPI Chip
#define CAN_INT 2 // interrupt pin
MCP_CAN CAN(PIN_SPI_CAN_CS); // Set CS Pin
#define DAQ_CAN_INTERVAL 100 // time in ms
uint16_t lastSendDaqMessage = millis();

// CAN Address
#define ID_BASE 0x01
#define ID_REAR_DAQ_DATA ID_BASE + 1 // send data on this address
#define ID_DASH_RIGHT_DATA 0x74 // receive data from dash
#define ID_FRONT_PEDALBOARD 0x37 // receive data from pedal

// Variables for CAN testing
// from pedal board
int wheel_left=0, wheel_right=0, damper_left=0, damper_right=0, steer=0, brake=0, pedal=0;


void filterCAN(unsigned long canID, unsigned char buf[8]){
  switch(canID){
    case ID_DASH_RIGHT_DATA:
      autoTemp = buf[2];
      // digitalWrite(PIN_FAN_OUT, autoTemp) // write fan output 'high' or 'low'
      break;
    case ID_FRONT_PEDALBOARD:
      wheel_left = buf[0];
      wheel_right= buf[1];
      damper_left= buf[2];
      damper_right= buf[3];
      steer= buf[4];
      brake= buf[5];
      pedal= buf[6];
      break;
  }
}

void setup() {
  // setup PinMode

  // IF USING INTERRUPT PIN UNCOMMENT THIS CODE:
  // pinMode(CAN_INT, INPUT)

  // Initialize CANbus Interface
  CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  CAN.setMode(MCP_NORMAL);

  Serial.begin(115200);
}

void loop() {

  // Initialize can buffers
  unsigned long id;
  unsigned char len = 0;
  unsigned char buf[8];

  // read the CANbus for data, filter as needed
  if (CAN_MSGAVAIL == CAN.checkReceive()){
    CAN.readMsgBuf(&id, &len, buf);
    filterCAN(id, buf);
  }

}
