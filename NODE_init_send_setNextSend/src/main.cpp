#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoLowPower.h>
#include <Wire.h>
#include "MAX30105.h"  //MAX3010x library
#include "heartRate.h" //Heart rate calculating algorithm


#define BAUD_RATE 115200

//#define ss 5
//#define rst 14
//#define dio0 2

//int Errorled = 6;

//-----------------HARDWARE declarations----------------
MAX30105 heartRateSensor;

const int GSR = A0; //analog pin 0
//------------------------------------------------------


//------------------------------------------SOFTWARE declarations--------------------------------
uint8_t myID = 0xA0;


//debug code
//int sendInterval = 0;
//int dataArray[] = {10, 20, 30, 40, 50};
//int arraySize = sizeof(dataArray) / sizeof(dataArray[0]);
//------------------------------------------------------


//****CHANGE THESE PARAMETERS (describe timings/#sensors)****
const byte amountOfHeartRateMeasurementsPerUnit = 5;
const byte amountOfSkinConductanceMeasurementsPerUnit = 10;
const byte amountOfSkinTempMeasurementsPerUnit = 10;
const int deltaMeasurementUnitsInSec = 60; // 60 seconds
const int deltaSendToGateInMinutes = -1; // -> this is set by the gate in function initWithGate()
const int amountOfSensors = 3;
//***********************************************************
//****DON'T CHANGE CALCULATED PARAMETERS****
const int deltaSendToGateInMilis = deltaSendToGateInMinutes*60000;
const int deltaMeasurementUnitsInMilis = deltaMeasurementUnitsInSec*1000;
const byte measurementUnitsBeforeSend = int(deltaSendToGateInMilis/deltaMeasurementUnitsInMilis);
float bufferToSend[measurementUnitsBeforeSend * amountOfSensors];
//******************************************
//-------------------------------------------------------------------------------------------------



//-----------------FUNCTION prototypes----------------
void initWithGateAuthenticate();
int waitForInterval();
int initWithGate();
void sendMeasurementsProper();
void sendMeasurementsString();

void performMeasurementsWithSleepInBetween();
float measurementUnitHeartRateSensor(byte);
float measurementUnitSkinConductanceSensor(byte);
float measurementUnitSkinTemperature(byte);
float measurementUnitMuscleTension(byte);

void ErrorLoRaBegin();
void setErrorLEDLow();
//------------------------------------------------------


void setup() {
  Serial.begin(BAUD_RATE);

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
    ErrorLoRaBegin();
  }

  deltaSendToGateInMinutes = initWithGate();
  //Serial.println("Intialization complete, send interval is:" + sendInterval);

  assert(deltaSendToGateInMinutes != -1);
}

void loop() {
  Serial.println("I will send in approximately" + sendInterval + "milliseconds");
  Serial.println("."); //serial.print don't always work
  performMeasurementsWithSleepInBetween();
  //sendMeasurementsProper();
  sendMeasurementsString():
}




void initWithGateAuthenticate() {
  LoRa.beginPacket();
  LoRa.write(myID);
  LoRa.endPacket();
  Serial.println("Sending done");
}


int waitForInterval() {
  String response = "";
  
  int interval = -1;

  // wait for the acklowledgement and interval from gate
  while (true) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // Lees het antwoord
      while (LoRa.available()) {
        response += (char)LoRa.read();
      }
      Serial.println("ACK + interval:" + response); //this should be my ID
      
      break;
    }
  }

  //parse the response into receivedID and interval
  if (response.length() > 0) {
    int commaIndex = response.indexOf(',');
    if (commaIndex != -1) {
      uint8_t id = (uint8_t)response.substring(0, commaIndex).toInt();
      if(id==myID){//could make further running of code depend on this to ensure it works right
        Serial.println("ACK works");
      }
      interval = response.substring(commaIndex + 1).toInt();
    }
  }
  return interval;
}



// send NODE ID to gate
int initWithGate(){
  Serial.println("Initializing with gate");
  initWithGateAuthenticate();
  return waitForInterval();
}



void sendMeasurementsProper(){
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
}


//needs parsing at the gateway
void sendMeasurementsString(){
  String dataString = "";
    
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
  //note that the bufferToSend is a float array, so we can store the heart rate and the skin conductance in the same array, thas why we multiply by 2
  //the buffer first stores #measurementUnitsBeforeSend heart rate measurements, then #measurementUnitsBeforeSend skin conductance measurements
  for(int i = 0; i < measurementUnitsBeforeSend; i++){
      bufferToSend[i] = measurementUnitHeartRateSensor(amountOfHeartRateMeasurementsPerUnit);
      bufferToSend[i + measurementUnitsBeforeSend] = measurementUnitSkinConductanceSensor(amountOfSkinConductanceMeasurementsPerUnit);
      bufferToSend[i + 2*measurementUnitsBeforeSend] = measurementUnitSkinTemperature(amountOfSkinTempMeasurementsPerUnit);
      LowPower.deepSleep(deltaMeasurementUnitsInMilis);
  }
}


//we can get stuck on this becuz no heartrate is measured (can be finicky)
//maybe add a timeout to this function, we send 0 if we dont get a heartrate in time
//we need to sum up the time we are delayed so we can send the data that much earlier
float measurementUnitHeartRateSensor(byte size){ //a unit is a measurement of x beats averaged
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
      
        if (beatsPerMinute < 255 && beatsPerMinute > 20){
          sum += beatsPerMinute;
          i++;
        }

      }
    }else{ //in case we dont detect x(size) beats in a row, we reset and measure again for x(size) beats.
      beatsPerMinute= 0;
      i=0;
    }
  }
  return sum/size;
}

//still needs testing
float measurementUnitSkinConductanceSensor(byte size){
  int sum;
  for(int i = 0;i<size;i++){
    sum += analogRead(GSR);
  }
  return sum/size;
}

//not sure if this works perfectly
float measurementUnitSkinTemperature(byte size){
  int sum;
  for(int i = 0; i<size;i++){
    sum += heartRateSensor.readTempC();
  }
  return sum/size;
}

float measurementUnitMuscleTension(byte size){
  int sum;
  for(int i = 0; i<size;i++){
    //sum += analogRead(GSR);
  }
  return sum/size;
}

//LED ERROR SIGNS
//incase LoRa.begin() fails, LED will stay high as long as it fails
void ErrorLoRaBegin(){
  //digitalWrite(Errorled, HIGH);
  //delay(2000);
}

void setErrorLEDLow(){
  //digitalWrite(Errorled, LOW);
}
