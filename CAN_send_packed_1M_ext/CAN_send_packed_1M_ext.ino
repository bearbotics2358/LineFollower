/* CAN Send Packed 1M Extended ID Example
 * This example sends a message every 10 msec
 * to test out the packed message structure and sending to the RoboRio
 * 
 * Code created for FeatherCAN M0
 * 
 * created 2/17/19 BD
 * 
 */

#include <mcp_can.h>
#include <SPI.h>

#define LED 13

// CAN0 RESET, INT and CS
#define CAN_RST 31   // MCP25625 RESET connected to Arduino pin 31
#define CAN0_INT 30  // Set INT to Arduino pin 30

MCP_CAN CAN0(8); // Set MCP25625 CS to Arduino pin 8

// CAN variables
unsigned long t_prev = 0;  // Variable to store last execution time
const unsigned int t_intvl = 10;  // Transmi interval in milliseconds
// CAN data to send - init w/ bogus data
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};



#define NUM_SENSORS 32   // number of sensors used

int sensorOutput[NUM_SENSORS];        // 1 or 0

// position of center of tape - 0 = centered on left most light sensor, 31 for right most sensor
int pos = 0;

// TOF sensor distance
int tof_distance = 100; // for testing

void test_values()
{
  int i;

  for(i = 0; i < NUM_SENSORS; i++) {
    if((i > 3) && (i < 9)) {
      sensorOutput[i] = 1;
    } else {
      sensorOutput[i] = 0;
    }
  }

  pos = 6;

  tof_distance = 149;
}


// pack message into format used by Line Follower to RoboRio
void packMsg()
{
  int i;
  long ltemp = 0;
  int distance = 0;

  // first, pack the sensorOutput, 1 bit per sensor, into the first 4 bytes of the message
  for(i = 0; i < NUM_SENSORS; i++) {
    // mask sensor Output to a single bit (0 or 1)
    // shift it into position - the left most center should be the MSb of a 32 bit number
    ltemp |= (sensorOutput[i] & 0x01) << (31 - i);          
  }
  data[0] = (ltemp >> 24) & 0x00ff;
  data[1] = (ltemp >> 16) & 0x00ff;
  data[2] = (ltemp >> 8) & 0x00ff;
  data[3] = ltemp & 0x00ff;

  // Then pack the distance from the center of the array to the center of the tape (line)
  // into the next 16 bits.
  // This is a signed value, in units of the distance between sensors, i.e, from
  // -16 to 16.  0 is excluded.
  if(pos < 16) {
    distance = pos - 16.0;
  } else {
    distance = pos - 15.0;
  }
  data[4] = (distance >> 8) & 0xff;
  data[5] = distance & 0xff;

  // Then pack the TOF distance into the next 16 bits.
  // This is a unsigned value, in units of mm
  data[6] = (tof_distance >> 8) & 0xff;
  data[7] = tof_distance & 0xff;
}

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);
  
  Serial.begin(115200);  // USB appears to ignore this and just run as fast as possible
  
  // so we can see the startup messages
  while(!Serial) ;

  // CAN chip RESET line
  pinMode(CAN_RST, OUTPUT);
  
  // reset CAN chip
  digitalWrite(CAN_RST, 0);
  delay(100);
  digitalWrite(CAN_RST, 1);
  delay(500);  

  // Initialize MCP25625 running at 16MHz with a baudrate of 1Mb/s and
  // the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
  }
  
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT_PULLUP);  // Configuring pin for /INT input
  
  digitalWrite(LED, 0);
}

void loop()
{
  // Send this at defined interval. 
  if(millis() >= t_prev + t_intvl) {
    t_prev = millis();

    test_values();

    // pack message for protocol from Feather CAN Line Follower to RoboRio
    packMsg();
    
    // send Extd msg
    byte sndStat = CAN0.sendMsgBuf(0x100 | 0x80000000, 1, 8, data);
    // byte sndStat = CAN_OK;
    
    /* debug printouts
    Serial.print("TEC: ");
    Serial.println(CAN0.errorCountTX());

    Serial.print("REC: ");
    Serial.println(CAN0.errorCountRX());
    */
    
    if(sndStat == CAN_OK)
      Serial.println("Message Sent Successfully!");
    else
      Serial.println("Error Sending Message...");
    Serial.println();
  }
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
