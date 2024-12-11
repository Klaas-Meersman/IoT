#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoLowPower.h>
#include <Wire.h>
#include "MAX30105.h"  //MAX3010x library
#include "heartRate.h" //Heart rate calculating algorithm


#define BAUD_RATE 115200
#define GSR_INPUT A0
#define EMG_INPUT A3


//#define ss 5
//#define rst 14
//#define dio0 2

int errorLED = 6;

//------------------------------------------HARDWARE declarations--------------------------------
MAX30105 heartRateSensor;
//-----------------------------------------------------------------------------------------------


//------------------------------------------SOFTWARE declarations--------------------------------
uint8_t myID = 0xA0; //=160

//***********EMG related declarations*******************************************
const int BUFFER_SIZE = 128;
int circular_buffer[BUFFER_SIZE];
int data_index, sumEMG;
int badPostureThreshold;
const int calibrationTime = 10000;
//******************************************************************************

//*************Heartrate related************************************************
unsigned long waistedTimeMeasuringHR;
//******************************************************************************

//**********CHANGE THESE PARAMETERS (describe timings/#sensors)*******************
const byte amountOfHeartRateMeasurementsPerUnit = 5;
const byte amountOfSkinConductanceMeasurementsPerUnit = 1;
const byte amountOfSkinTempMeasurementsPerUnit = 1;
const byte amountOfMuscleTensionMeasurementsPerUnit = 1;
const int deltaMeasurementUnitsInSec = 60; // seconds //default: 60sec
unsigned long deltaSendToGateInMilis = -1; //this is set by initialziing with gate
const int amountOfSensors = 4; //heart rate, skin conductance, skin temperature, muscle tension
//********************************************************************************
//*************DON'T CHANGE CALCULATED PARAMETERS*********************************
const int MAXBufferSize = 240;
int deltaMeasurementUnitsInMilis = deltaMeasurementUnitsInSec*1000;
int measurementUnitsBeforeSend = int(deltaSendToGateInMilis/deltaMeasurementUnitsInMilis);
float bufferToSend[MAXBufferSize];
int bufferSize = measurementUnitsBeforeSend * amountOfSensors;
//********************************************************************************
//-------------------------------------------------------------------------------------------------

//-----------------FUNCTION prototypes----------------
void signUpToGate();
unsigned long waitForGateAndGetInterval();
unsigned long initWithGate();
void sendMeasurementsProper();
void sendMeasurementsString();
void calculateParameters();

void performMeasurementsWithSleepInBetween();
float measurementUnitHeartRateSensor(byte);
float measurementUnitSkinConductanceSensor(byte);
float measurementUnitSkinTemperature(byte);
float measurementUnitMuscleTension(byte);

//EMG related functions
bool calibrateEMG();
float measurementUnitMuscleTension(byte);
float EMGFilter(float);
int getEnvelop(int);
bool checkPosture(int);

void customDelay(unsigned long);

//LED signs
void errorLoRaBegin();
void LEDLow();
void LEDHigh();
void LEDsendingMessage();
//------------------------------------------------------


void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(GSR_INPUT, INPUT);
  pinMode(EMG_INPUT, INPUT);
  pinMode(errorLED, OUTPUT);
  LEDHigh();
  calibrateEMG(); 

  heartRateSensor.begin(Wire, I2C_SPEED_FAST); // Use default I2C port, 400kHz speed
  heartRateSensor.setup();                     // Configure sensor with default settings
  heartRateSensor.setPulseAmplitudeRed(0x0A);  // Turn Red LED to low to indicate sensor is running
  
  while (!Serial);
  Serial.println("LoRa Sender");
  //LoRa.setPins(ss, rst, dio0);
  //pinMode(Errorled, OUTPUT);
  //setErrorLEDLow();

  while (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    errorLoRaBegin();
  }

  deltaSendToGateInMilis = initWithGate();
  //Serial.println("Intialization complete, send interval is:" + (String)deltaSendToGateInMinutes);
  calculateParameters();
}

void loop() {
  performMeasurementsWithSleepInBetween();
  //sendMeasurementsProper();
  sendMeasurementsString();
}

void calculateParameters(){
  deltaMeasurementUnitsInMilis = deltaMeasurementUnitsInSec*1000;
  measurementUnitsBeforeSend = int(deltaSendToGateInMilis/deltaMeasurementUnitsInMilis);
  bufferSize = measurementUnitsBeforeSend * amountOfSensors;
  Serial.println("The buffer size is " + String(bufferSize));
  Serial.println("Delta measurements is " + String(deltaMeasurementUnitsInMilis));
  Serial.println(deltaSendToGateInMilis);
}



void signUpToGate() {
  char hexBuffer[2]; 
  sprintf(hexBuffer, "%02X", myID); 
  String IDstring = String(hexBuffer);

  LoRa.beginPacket();
  LoRa.print(IDstring);
  LoRa.endPacket();
  Serial.println("Sending done");
}


unsigned long waitForGateAndGetInterval() {
  String response = "";
  unsigned long interval = -1;
  unsigned long setupOfGateLeft = 0;

  // Wait for a message (unchanged)
  while (true) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      while (LoRa.available()) {
        response += (char)LoRa.read();
      }
      Serial.println("ACK + interval + setupOfGateLeft: " + response);
      break;
    }
  }



  //parse response to get ID, interval and setupOfGateLeft
  if (response.length() > 0) {
    int firstComma = response.indexOf(',');
    int secondComma = response.indexOf(',', firstComma + 1);
  
    if (firstComma != -1 && secondComma != -1) {
      uint8_t id = (uint8_t)response.substring(0, firstComma).toInt();
      if (id == myID) {
        Serial.println("ACK works, id confirmed");
      }
      interval = (unsigned long)atol(response.substring(firstComma + 1, secondComma).c_str());
      setupOfGateLeft = (unsigned long)atol(response.substring(secondComma + 1).c_str());
    
      Serial.println("Interval: " + String(interval));
      Serial.println("SetupOfGateLeft: " + String(setupOfGateLeft));
    }
  }
  Serial.print("We wait " + String(setupOfGateLeft));
  Serial.println(" until gate is ready");
  //set led low, initiating is done
  LEDLow();
  //LowPower.deepSleep(setupOfGateLeft);
  customDelay(setupOfGateLeft);
  Serial.println("Gate is ready");
  return interval;
}


//initiates the node with the gate, returns the interval we need to send data, wait for the gate to be ready
unsigned long initWithGate(){
  Serial.println("Initializing with gate");
  signUpToGate();
  return waitForGateAndGetInterval();
}

//not used atm
/* void sendMeasurementsProper(){
  //------debug code------
  Serial.print("Sending measurements: ");
  for(int i = 0; i < arraySize; i++){
    Serial.print(dataArray[i]);
    Serial.print(" ");
  }
  Serial.println();
  //----------------------

  LoRa.beginPacket();
  //first let them know who is sending
  LoRa.write(myID);
  //send data
  int lengthOfArray = sizeof(bufferToSend)/sizeof(bufferToSend[0]);
  for (int i = 0; i < lengthOfArray; i++) {
    LoRa.write(bufferToSend[i]);
  }
  LoRa.endPacket();
} */


//needs parsing at the gateway
void sendMeasurementsString(){
  char hexBuffer[2];
  sprintf(hexBuffer, "%02X", myID);
  String dataString = String(hexBuffer);
  dataString += ","; // Add delimiter
    
    // Convert array to string
    for (int i = 0; i < bufferSize; i++) {
        dataString += String(bufferToSend[i]);
        if (i < bufferSize - 1) {
            dataString += ","; // Add delimiter
        }
    }
    Serial.print("Sent: ");
    Serial.println(dataString);
    // Send the string
    LoRa.beginPacket();
    LoRa.print(dataString);
    LoRa.endPacket();
}

void performMeasurementsWithSleepInBetween(){
  //note that the bufferToSend is a float array, so we can store the heart rate, skin conductance.... in the same array, thas why we multiply by 2,3,4
  //the buffer first stores #measurementUnitsBeforeSend heart rate measurements, then #measurementUnitsBeforeSend skin conductance measurements
  for(int i = 0; i < measurementUnitsBeforeSend; i++){
      bufferToSend[i] = measurementUnitHeartRateSensor(amountOfHeartRateMeasurementsPerUnit);
      bufferToSend[i + measurementUnitsBeforeSend] = measurementUnitSkinConductanceSensor(amountOfSkinConductanceMeasurementsPerUnit);
      bufferToSend[i + 2 * measurementUnitsBeforeSend] = measurementUnitSkinTemperature(amountOfSkinTempMeasurementsPerUnit);
      bufferToSend[i + 3 * measurementUnitsBeforeSend] = measurementUnitMuscleTension(amountOfMuscleTensionMeasurementsPerUnit);
      LEDsendingMessage();
      //LowPower.deepSleep(deltaMeasurementUnitsInMilis -  waistedTimeMeasuringHR);
      customDelay(deltaMeasurementUnitsInMilis- waistedTimeMeasuringHR );
      
  }
}

//we can get stuck on this becuz no heartrate is measured (can be finicky)
//timeout to this function, we send 0 if we dont get a heartrate in time
//we need to measure the time we are delayed by measuring the heartrate so we can sleep less in between measurements to send at right times
//we only do this for heartrate as it takes significantly longer than any other measurement, it also varies in time
float measurementUnitHeartRateSensor(byte size){ //a unit is a measurement of x beats averaged
  waistedTimeMeasuringHR = 0;
  long waistedTimeMeasuring_1 = millis();
  int measuringRestarted = 0;
  long lastBeat = 0; // Time at which the last beat occurred
  float beatsPerMinute;
  float sum;
  int i = 0;
  while(i<size){
    long irValue = heartRateSensor.getIR(); 
    if (irValue > 80000)  {                                                     
      if (checkForBeat(irValue) == true) {
        long delta = millis() - lastBeat; 
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0); 
        Serial.println("Beats/min" + String(beatsPerMinute));
      
        if (beatsPerMinute < 255 && beatsPerMinute > 20){
          sum += beatsPerMinute;
          i++;
        }

      }
    }else{ //in case we dont detect x(size) beats in a row, we reset and measure again for x(size) beats.
      beatsPerMinute= 0;
      if(i>0){
        measuringRestarted++;
      }
      i=0;
      
      long waistedTimeMeausring_2 = millis();
      
      //to avoid infinite measuring when no heartrate detected
      if(waistedTimeMeausring_2 - waistedTimeMeasuring_1 > 10000 || measuringRestarted > 3){
        Serial.println("Didn't detect heartrate");
        waistedTimeMeasuringHR = millis() - waistedTimeMeasuring_1;
        measuringRestarted=0;
        return 0;
      }
    }
  }
  //set waistedTimeMeasuring so we can substract it from the sleep time, so we still send at right moment
  waistedTimeMeasuringHR = millis() - waistedTimeMeasuring_1;
  Serial.println("Sleeping less: " + String(waistedTimeMeasuringHR));
  return sum/size;
}

//still needs testing
float measurementUnitSkinConductanceSensor(byte size){
  float sum;
  for(int i = 0;i<size;i++){
    sum += analogRead(GSR_INPUT);
  }
  return sum/size;
}

//not sure if this works perfectly
float measurementUnitSkinTemperature(byte size){
  float sum;
  for(int i = 0; i<size;i++){
    sum += heartRateSensor.readTemperature()-5;
  }
  Serial.println("Temperature measured: " + String(sum/size));
  return sum/size;
}


float measurementUnitMuscleTension(byte size) {
  int badPostureCount = 0;
  for (int i = 0; i < size; i++) {
    int sensor_value = analogRead(EMG_INPUT);
    int signal = EMGFilter(sensor_value);
    int envelop = getEnvelop(abs(signal));
    if (checkPosture(envelop)) {
      badPostureCount++;
      }
    }
    float badPosturePercentage = (badPostureCount / (float)size) * 100.0;
    return badPosturePercentage;
}

//EMG related functions
void calibrateSensor() {
    unsigned long startTime = millis();
    int filteredSignal;
    int evelopeValue;
    int minEvelope = 9999;
    int maxEvelope = 0;
    while (millis() - startTime < calibrationTime) {
        int rawValue = analogRead(EMG_INPUT);

        filteredSignal = EMGFilter(rawValue);
        evelopeValue = getEnvelop(abs(filteredSignal));
        
        // Update baseline (minimum) and max values
        if (evelopeValue < minEvelope) {
            minEvelope = evelopeValue;
        }
        if (evelopeValue > maxEvelope) {
            maxEvelope = evelopeValue;
        }
        
        // Display progress
        Serial.print("Calibrating... Min: ");
        Serial.print(minEvelope);
        Serial.print(" Max: ");
        Serial.println(maxEvelope);
        delay(5);
    }
    int envelopeRange = maxEvelope - minEvelope;
    badPostureThreshold = minEvelope + (0.1 * envelopeRange);
}

float EMGFilter(float input){
  float output = input;
  {
    static float z1, z2; // filter section state
    float x = output - 0.05159732*z1 - 0.36347401*z2;
    output = 0.01856301*x + 0.03712602*z1 + 0.01856301*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -0.53945795*z1 - 0.39764934*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - 0.47319594*z1 - 0.70744137*z2;
    output = 1.00000000*x + 2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  {
    static float z1, z2; // filter section state
    float x = output - -1.00211112*z1 - 0.74520226*z2;
    output = 1.00000000*x + -2.00000000*z1 + 1.00000000*z2;
    z2 = z1;
    z1 = x;
  }
  return output;
}

int getEnvelop(int abs_emg){
	sumEMG -= circular_buffer[data_index];
	sumEMG += abs_emg;
	circular_buffer[data_index] = abs_emg;
	data_index = (data_index + 1) % BUFFER_SIZE;
	return (sumEMG/BUFFER_SIZE) * 2;
}

bool checkPosture(int envelope) {
    if (envelope > badPostureThreshold) {
        return true; // Bad posture detected
    }
    return false; // Good posture
}


void customDelay(unsigned long delayTime) {
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime) {
  }
}


//LED ERROR SIGNS
void errorLoRaBegin(){
  for(int i = 0; i < 5; i++){
    LEDLow();
    customDelay(500);
    LEDHigh();
    customDelay(500);
  }
}

void LEDLow(){
  digitalWrite(Errorled, HIGH);
}

void LEDHigh(){
  digitalWrite(Errorled, LOW);
}

void LEDsendingMessage(){
  for(int i = 0; i < 5; i++){
    LEDLow();
    customDelay(100);
    LEDHigh();
    customDelay(100);
  }
}
