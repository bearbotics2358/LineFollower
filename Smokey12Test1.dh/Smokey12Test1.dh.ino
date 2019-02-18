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

#define NUM_SENSORS            32   // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define FOLLOWER_ENABLE A0
#define LEFT_MUX_SIGNAL A1
#define RIGHT_MUX_SIGNAL A2
#define LED 13
#define TX_INTERVAL 10
int leftPins[] = {5, 6, 9, 10};
int rightPins[] = {0, 1, 12, 11};
int thresholds[] = {879, 1064, 1033, 1054, 1063, 1057, 1028, 1066, 956, 993, 1013, 1058, 1054, 1057, 1068, 1007, 932, 1067, 1069, 1047, 1061, 1008, 1039, 1003, 985, 1068, 1033, 1040, 1026, 1060, 1063, 1119
};

unsigned long prevTX = 0;
unsigned long tnow;

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
  delay(500);
  Serial.begin(9600); // set  data rate in bits per second for serial data transmission

  // initialize pins
  pinMode(LED, OUTPUT);
  pinMode(FOLLOWER_ENABLE, OUTPUT);
  digitalWrite(FOLLOWER_ENABLE, 1); 
  //setDimmingLevel(0);
  digitalWrite(LED, 1);
  
// wait for serial port
  while(!Serial) ;
  
  for(i = 0; i < 4; i++) {
    pinMode(leftPins[i], OUTPUT);
    pinMode(rightPins[i], OUTPUT);
  }

  setLeftPins(0);
  setRightPins(0);

  delay(500);

  prevTX = millis();

  digitalWrite(LED, 0);
}


void loop()
{
  // is it time to take a reading?
  if(millis() < prevTX + TX_INTERVAL) {
    return;
  }

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
    if (a_temp > thresholds[chan]){
       sensorOutput[chan]=0;
     }
     else {
       sensorOutput[chan]=1;
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
       }
     else {
       sensorOutput[chan + 16] = 1;
     }

    sensorValues[chan + 16] = a_temp;
    setRightPins(nextChan);

    chan = nextChan;
    delay(100);
  } while(chan != 0);

  int sumoftorque = 0;
  int mass =0;

  for(i = 0; i < (NUM_SENSORS); i++) {
    sumoftorque += (sensorOutput[i] * (i));
    mass += sensorOutput[i];
    }
  pos = sumoftorque/mass;
  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  { 
    // Serial.print(i);
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
   }
   Serial.println();

  for (unsigned char k = 0; k < NUM_SENSORS; k++)
  {
    //Serial.print(k);
    Serial.print(sensorOutput[k]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
   }
  
  Serial.println();
  Serial.print(pos);
  Serial.println();
  delay(250);
}
