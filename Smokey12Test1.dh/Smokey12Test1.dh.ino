
// This example is designed for use with six QTR-1A sensors or the first six sensors of a
// QTR-8A module.  These reflectance sensors should be connected to analog inputs 0 to 5.
// The QTR-8A's emitter control pin (LEDON) can optionally be connected to digital pin 2,
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to
// QTR_NO_EMITTER_PIN.
// The main loop of the example reads the raw sensor values (uncalibrated).
// You can test this by taping a piece of 3/4" black electrical tape to a piece of white
// paper and sliding the sensor across it.  It prints the sensor values to the serial
// monitor as numbers from 0 (maximum reflectance) to 1023 (minimum reflectance).

// #include <QTRSensors.h>
#include <mcp_can.h>
#include <SPI.h>

#define NUM_SENSORS            32   // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading

// define pins
#define FOLLOWER_ENABLE A0
#define LEFT_MUX_SIGNAL A1
#define RIGHT_MUX_SIGNAL A2

#define LED 13

// CAN0 RESET, INT
#define CAN0_RST 31   // MCP25625 RESET connected to Arduino pin 31
#define CAN0_INT 30  // Set INT to Arduino pin 30

// CAN variables
unsigned long t_prev = 0;  // Variable to store last execution time
const unsigned int t_intvl = 10;  // Transmi interval in milliseconds
// CAN data to send - init w/ bogus data
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};


/*
The Manufacturer (8 bits)
The Device Type (5 bits)
An API ID (10 bits)
The Device ID (6 bits)

For Team created modules, FIRST says that 
Manufacturer - HAL_CAN_Man_kTeamUse = 8
Device Type - HAL_CAN_Dev_kMiscellaneous = 10
API ID is up to us
Device ID is unique to each module of a specific type (e.g., we can have more than 1 line follower)

CAN ID: (Device Type): 01010 (Mfr ID): 0000 1000 (API ID): 00 0000 0000 (Device ID):00 0001 
CAN ID: 01010 0000 1000 00 0000 0000 00 0001 
CAN ID: 0 1010 0000 1000 0000 0000 0000 0001 
which is: 0x0A080001

*/
#define CAN_ID 0x0a080001

MCP_CAN CAN0(8); // Set MCP25625 CS to Arduino pin 8

// Line Follower variables
int leftPins[] = {5, 6, 9, 10};
int rightPins[] = {0, 1, 12, 11};
int thresholds[] = {903, 809, 774, 768, 797, 788, 784, 809, 713, 752, 751, 803, 796, 800, 801, 741, 706, 795, 810, 784, 789, 737, 751, 736, 743, 806, 766, 755, 739, 765, 763, 843
};

// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
// QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5},
//  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR)
int sensorValues[NUM_SENSORS];        // analog readings
int sensorOutput[NUM_SENSORS];        // 1 or 0
int reference[NUM_SENSORS];           // ideal sensor output
long correlationResults[NUM_SENSORS];  

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

  // data[4] = (pos >> 8) & 0xff;
  // data[5] = pos & 0xff;

  // Then pack the TOF distance into the next 16 bits.
  // This is a unsigned value, in units of mm
  data[6] = (tof_distance >> 8) & 0xff;
  data[7] = tof_distance & 0xff;
}

void setLeftPins(int chan)
{
  digitalWrite(leftPins[0], chan & 0x01);
  digitalWrite(leftPins[1], (chan & 0x02) >> 1);
  digitalWrite(leftPins[2], (chan & 0x04) >> 2);
  digitalWrite(leftPins[3], (chan & 0x08) >> 3);
  // Serial.print("left");
  // Serial.print(chan);
}

void setRightPins(int chan)
{
  digitalWrite(rightPins[0], chan & 0x01);
  digitalWrite(rightPins[1], (chan & 0x02) >> 1);
  digitalWrite(rightPins[2], (chan & 0x04) >> 2);
  digitalWrite(rightPins[3], (chan & 0x08) >> 3);
  // Serial.print("right");
  // Serial.print(chan);
}

void setup()
{
  int i;

  // configure LED pin and turn LED on
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);
  
  delay(500);
  Serial.begin(115200);  // USB appears to ignore this and just run as fast as possible

  // initialize pins
  pinMode(FOLLOWER_ENABLE, OUTPUT);
  digitalWrite(FOLLOWER_ENABLE, 1); 
  //setDimmingLevel(0);
  
  // wait for serial port
  // while(!Serial) ;
  
  // configure channels select outputs to multiplexers
  for(i = 0; i < 4; i++) {
    pinMode(leftPins[i], OUTPUT);
    pinMode(rightPins[i], OUTPUT);
  }

  setLeftPins(0);
  setRightPins(0);

  // CAN chip RESET line
  pinMode(CAN0_RST, OUTPUT);
  // Configuring pin for /INT input
  pinMode(CAN0_INT, INPUT_PULLUP);
  
  // reset CAN chip
  digitalWrite(CAN0_RST, 0);
  delay(100);
  digitalWrite(CAN0_RST, 1);
  delay(500);  

  // Initialize MCP25625 running at 16MHz with a baudrate of 1Mb/s and
  // the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP25625 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP25625...");
  }

  Serial.println("Line Follower");
  
  CAN0.setMode(MCP_NORMAL);

  delay(500);

  digitalWrite(LED, 0);
}


void loop()
{
  // is it time to take and send a reading?
  if(millis() < t_prev + t_intvl) {
    return;
  }
  t_prev = millis();

  int nextChan = 0;
  int chan = 0;
  int a_temp = 0;
  int b_temp = 0;
  int i;
  int j;
  int k;
  
  do {

    // read raw sensor values
    //setLeftPins(8);
  
    // read left sensor channel
    a_temp = 0;
    for(i = 0; i < NUM_SAMPLES_PER_SENSOR; i++) {
    // for(i = 0; i < 100; i++) {
    //while (1) {
      //b_temp = analogRead(LEFT_MUX_SIGNAL);
      //Serial.println(b_temp);
      //delay(100);
      a_temp += analogRead(LEFT_MUX_SIGNAL);
      // Serial.print("left");
      // Serial.print(i);
      // Serial.print(b_temp);
      // Serial.print(a_temp);
      // Serial.println();
    }
    //while(1) ;
    a_temp /= NUM_SAMPLES_PER_SENSOR;

     //if(a_temp > 1000) {
    if(a_temp > thresholds[chan]){
       sensorOutput[chan] = 0;
    } else {
       sensorOutput[chan] = 1;
    }

    sensorValues[chan] = a_temp;

    // change left mux channel to next sensor
    // calculate next channel
    nextChan = chan + 1;
    // Serial.print(chan);
    if(nextChan > 15) 
    {
      nextChan = 0;
    }
    setLeftPins(nextChan);

    // read right sensor channel
    a_temp = 0;
    for(i = 0; i < NUM_SAMPLES_PER_SENSOR; i++) {
      // b_temp = analogRead(RIGHT_MUX_SIGNAL);
      a_temp += analogRead(RIGHT_MUX_SIGNAL);
      // Serial.print("right");
      // Serial.print(i);
      // Serial.print(b_temp);
      // Serial.print(a_temp);
      // Serial.println();
    }
    a_temp /= NUM_SAMPLES_PER_SENSOR;
    
    // if(a_temp > 1000) {
    if (a_temp > thresholds[chan + 16]){
      sensorOutput[chan + 16] = 0;
    } else {
      sensorOutput[chan + 16] = 1;
    }

    sensorValues[chan + 16] = a_temp;
    setRightPins(nextChan);

    chan = nextChan;
    // delay(100);
  } while(chan != 0);

  int sumoftorque = 0;
  int mass = 0;

  for(i = 0; i < (NUM_SENSORS); i++) {
    sumoftorque += (sensorOutput[i] * (i));
    mass += sensorOutput[i];
  }
  pos = sumoftorque/mass;
  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++) { 
    // Serial.print(i);
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();

  for (unsigned char k = 0; k < NUM_SENSORS; k++) {
    // Serial.print(k);
    // Serial.print(sensorOutput[k]);
    // Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  
  // Serial.println();
  Serial.print(pos);
  Serial.println();

  // for testing, especially when no array is present
  // test_values();

  // pack message for protocol from Feather CAN Line Follower to RoboRio
  packMsg();
    
  // send Extended msg
  // byte sndStat = CAN0.sendMsgBuf(CAN_ID | 0x80000000, 1, 8, data);
  byte sndStat = CAN0.sendMsgBuf(CAN_ID, 1, 8, data);
  /*
  for(i = 4; i < 6; i++) {
    Serial.println(data[i]);
  }
  */
  // byte sndStat = CAN_OK;
    
  /* debug printouts
  Serial.print("TEC: ");
  Serial.println(CAN0.errorCountTX());

  Serial.print("REC: ");
  Serial.println(CAN0.errorCountRX());
  */
  /*
  if(sndStat == CAN_OK) {
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }
  */
  // Serial.println();
  
}
