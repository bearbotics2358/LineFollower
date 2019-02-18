/* CAN Receive and Decode Line Follower message

created 2/17/19 BD

*/

#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

unsigned long prevTX = 0;          // Variable to store last execution time
const unsigned int invlTX = 1000;  // One second interval constant

#define CAN0_INT 30                // Set INT to pin PB22 (TXD for Arduino, RXD_INT for mine)
MCP_CAN CAN0(8);                   // Set CS to pin 8

// MCP25625 RESET
#define TXD_RST 31

#define LED 13

int loop_cnt = 0;

byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};


/*
Calculate CAN ID for Line Follower msg transmission

The Manufacturer (8 bits)
The Device Type (5 bits)
An API ID (10 bits)
The Device ID (6 bits)

For Team created modules, FIRST says that 
Manufacturer - HAL_CAN_Man_kTeamUse = 8
Device Type - HAL_CAN_Dev_kMiscellaneous = 10
API ID is up to us
Device ID is unique to each module of a specific type (e.g., we can have more than 1 line follower)

CAN ID: (Mfr ID): 0000 1000  (Device Type): 01010  (API ID): 00 0000 0000 (Device ID):00 0001 
CAN ID: 0 0001 0000 1010   0000 0000   0000 0001 
which is: 0x010a0001
*/
#define CAN_ID 0x010a0001
// CAN MASK is 1 bits in all 29 bit positions, except for the Device ID
#define CAN_MASK 0x01ffffc0
// all CAN followers will have the following result when ANDed with the CAN_MASK
#define CAN_FOLLOWER 0x010a0000

#define NUM_SENSORS            32   // number of sensors used
int sensorOutput[NUM_SENSORS];        // 1 or 0

// position of center of tape in inches
float fpos = 0;

// TOF sensor distance
int tof_distance = 0;

void decodeLineFollowerMsg()
{
  int i;
  int j;
  char stemp[80];
  int16_t i16;

  // decode sensorOutput[] from bytes 0-3
  for(i = 0; i < 4; i++)
  {
    // get 8 sensor values from each byte
    for(j = 0; j < 8; j++) {
      sensorOutput[i * 8 + j] = (rxBuf[i] >> (7 - j)) & 0x01;
    }
  }
  for(i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorOutput[i]);
    Serial.print(" ");
  }
  Serial.println();

  // decode position
  // first, get the value as sent
  i16 = (rxBuf[4] << 8) | rxBuf[5];
  sprintf(stemp, "pos (as sent): %d", i16);
  Serial.println(stemp);

  fpos =  (i16 * 8.0)/25.4; // convert to inches - 8mm per sensor
  Serial.print("pos (inches): ");
  Serial.println(fpos);

  // decode time of flight distance
  tof_distance = (rxBuf[6] << 8) | rxBuf[7];
  sprintf(stemp, "tof_distance: %d", tof_distance);
  Serial.println(stemp);
  Serial.println();
}

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);
  
  Serial.begin(115200);

  // so we can see the startup messages
  while(!Serial) ;

  Serial.println("starting...");

  // CAN chip RESET line
  pinMode(TXD_RST, OUTPUT);
  
  // reset CAN chip
  digitalWrite(TXD_RST, 0);
  delay(100);
  digitalWrite(TXD_RST, 1);
  delay(500);  

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);          // Set operation mode to normal so the MCP2515 sends acks to received data.
  // CAN0.setMode(MCP_LISTENONLY);

  // enable SOF output on CLKOUT for logic analyzer
  CAN0.setSOF_output();
  
  pinMode(CAN0_INT, INPUT_PULLUP); // Configuring pin for /INT input

  Serial.println("CAN receive and decode line follower msg...");

  Serial.print("CAN0_INT: ");
  Serial.println(digitalRead(CAN0_INT));
  
  delay(1000);
  digitalWrite(LED, 0);
}

void loop()
{
  int ret; 

  // Serial.print("REC: ");
  // Serial.println(CAN0.errorCountRX());
    
  // Serial.print("CAN0_INT: ");
  // Serial.println(digitalRead(CAN0_INT));
  
  
  ret = digitalRead(CAN0_INT);

  // if(ret == CAN_OK)
  if(ret == 0) {
    // Serial.println("msg recvd");
    
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    if(rxId != 0x00000000) {
      // got something

      if((rxId & 0x80000000) == 0x80000000) {    // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      } else {
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
      }
      Serial.print(msgString);
  
      if((rxId & 0x40000000) == 0x40000000) {    // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        Serial.print(msgString);
      } else {
        for(byte i = 0; i<len; i++) {
          sprintf(msgString, " 0x%.2X", rxBuf[i]);
          Serial.print(msgString);
        }
      }
      Serial.println();

      if(((rxId & CAN_ID) & CAN_MASK) == CAN_FOLLOWER) {
        // message from a line follower
        decodeLineFollowerMsg();
      }
    }
  }

}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
