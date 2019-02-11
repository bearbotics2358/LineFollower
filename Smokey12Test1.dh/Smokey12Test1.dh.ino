#include <QTRSensors.h>

// This example is designed for use with six QTR-1A sensors or the first six sensors of a
// QTR-8A module.  These reflectance sensors should be connected to analog inputs 0 to 5.
// The QTR-8A's emitter control pin (LEDON) can optionally be connected to digital pin 2,
// or you can leave it disconnected and change the EMITTER_PIN #define below from 2 to
// QTR_NO_EMITTER_PIN.

// The main loop of the example reads the raw sensor values (uncalibrated).
// You can test this by taping a piece of 3/4" black electrical tape to a piece of white
// paper and sliding the sensor across it.  It prints the sensor values to the serial
// monitor as numbers from 0 (maximum reflectance) to 1023 (minimum reflectance).


#define NUM_SENSORS             32 // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading

#define FOLLOWER_ENABLE A2
#define LEFT_MUX_SIGNAL A0
#define RIGHT_MUX_SIGNAL A1
#define LED 13

#define TX_INTERVAL 10


int leftPins[] = {5, 6, 9, 10};

int rightPins[] = {0, 1, 12, 11};

unsigned long prevTX = 0;

unsigned long tnow;
 
// sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
// QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5},
//  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR);
int sensorValues[NUM_SENSORS];    // analog readings
int sensorOutput[NUM_SENSORS];    // 1 or 0

setLeftPins(int chan)
{
  digitalWrite(leftPins[0], chan & 0x01);
  digitalWrite(leftPins[1], (chan & 0x02) >> 1);
  digitalWrite(leftPins[2], (chan & 0x04) >> 2);
  digitalWrite(leftPins[3], (chan & 0x08) >> 3);
}

setRightPins(int chan)
{
  digitalWrite(rightPins[0], chan & 0x01);
  digitalWrite(rightPins[1], (chan & 0x02) >> 1);
  digitalWrite(rightPins[2], (chan & 0x04) >> 2);
  digitalWrite(rightPins[3], (chan & 0x08) >> 3);
}

void setup()
{
  int i;
  the
  delay(500);
  Serial.begin(9600); // set  data rate in bits per second for serial data transmission
  
  // initialize pins
  pinMode(LED, OUTPUT);
  pinMode(FOLLOWER_ENABLE, OUTPUT);
  digitalWrite(FOLLOWER_ENABLE, 0);
  
  for(i = 0; i < 4; i++) {
    pinMode(leftPins[i], OUTPUT);
    pinMode(rightPins[i], OUTPUT);
  }

  setLeftPins(0);
  setRightPins(0);

  delay(500);

  prevTX = millis();

}


void loop()
{
  // is it time to take a reading?
  if(millis() < prevTX + TX_INTERVAL) {
    return;
  }

  int nextChan;
  int a_temp = 0;
  int i;
  
  do {
  
    // read raw sensor values
  
    // read left sensor channel
    for(i = 0; i < NUM_SAMPLES_PER_SENSOR; i++) {
      a_temp += analogRead(LEFT_MUX_SIGNAL);
    }
    a_temp /= NUM_SAMPLES_PER_SENSOR;

    sensorValues[chan] = a_temp;

    // change left mux channel to next sensor
    // calculate next channel
    nextChan = chan + 1;
    if(nextChan > 15) {
      nextChan = 0
    }
    setLeftPins(nextChan);

    // read right sensor channel
    for(i = 0; i < NUM_SAMPLES_PER_SENSOR; i++) {
      a_temp += analogRead(RIGHT_MUX_SIGNAL);
    }
    a_temp /= NUM_SAMPLES_PER_SENSOR;

    sensorValues[chan + 16] = a_temp;
    setRightPins(nextChan);

    chan = nextChan;
  } while(chan != 0);
  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();

  delay(250);
}
