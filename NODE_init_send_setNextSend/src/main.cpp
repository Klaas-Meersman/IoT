#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoLowPower.h>
#include <Wire.h>
#include "MAX30105.h"  //MAX3010x library
#include "heartRate.h" //Heart rate calculating algorithm


#define BAUD_RATE 115200
#define EMG_INPUT A0
#define GSR_INPUT A1

//#define ss 5
//#define rst 14
//#define dio0 2

//int Errorled = 6;

//------------------------------------------HARDWARE declarations--------------------------------
MAX30105 heartRateSensor;

//const int GSR = A0; //analog pin 0 -> defines zouden dit moeten al moeten doen

//-----------------------------------------------------------------------------------------------


//------------------------------------------SOFTWARE declarations--------------------------------
uint8_t myID = 0xA0;


//EMG related declarations
const int BUFFER_SIZE = 128;
int circular_buffer[BUFFER_SIZE];
int data_index, sumEMG;
int badPostureThreshold;
//EMG

//debug code-------------------------------------------
//int sendInterval = 0;
//int dataArray[] = {10, 20, 30, 40, 50};
//int arraySize = sizeof(dataArray) / sizeof(dataArray[0]);
//------------------------------------------------------


//**********CHANGE THESE PARAMETERS (describe timings/#sensors)*******************
const byte amountOfHeartRateMeasurementsPerUnit = 5;
const byte amountOfSkinConductanceMeasurementsPerUnit = 10;
const byte amountOfSkinTempMeasurementsPerUnit = 10;
const byte amountOfMuscleTensionMeasurementsPerUnit = 10;
const int deltaMeasurementUnitsInSec = 60; // 60 seconds
int deltaSendToGateInMinutes = -1; // -> this is set by the gate in function initWithGate()
const int amountOfSensors = 4; //heart rate, skin conductance, skin temperature, muscle tension
//********************************************************************************
//*************DON'T CHANGE CALCULATED PARAMETERS*********************************
const int deltaSendToGateInMilis = deltaSendToGateInMinutes*60000;
const int deltaMeasurementUnitsInMilis = deltaMeasurementUnitsInSec*1000;
const byte measurementUnitsBeforeSend = int(deltaSendToGateInMilis/deltaMeasurementUnitsInMilis);
float bufferToSend[measurementUnitsBeforeSend * amountOfSensors];
//********************************************************************************
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

//EMG related functions
bool calibrateEMG();
float measurementUnitMuscleTension(byte);
float EMGFilter(float);
int getEnvelop(int);
bool checkPosture(int);

//error signs
void ErrorLoRaBegin();
void setErrorLEDLow();
//------------------------------------------------------


void setup() {
  Serial.begin(BAUD_RATE);
  pinmMode(GSR_INPUT, INPUT);
  pinMode(EMG_INPUT, INPUT);
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


//not used atm
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
      bufferToSend[i + 2 * measurementUnitsBeforeSend] = measurementUnitSkinTemperature(amountOfSkinTempMeasurementsPerUnit);
      bufferToSend[i + 3 * measurementUnitsBeforeSend] = measurementUnitMuscleTension(amountOfMuscleTensionMeasurementsPerUnit);
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
  return sum/ size;
}


float measurementUnitMuscleTension(byte size) {
    int sum = 0;
    //int badPostureCount = 0;
    for (int i = 0; i < size; i++) {
        int sensor_value = analogRead(EMG_INPUT);
        int signal = EMGFilter(sensor_value);
        int envelop = getEnvelop(abs(signal));
        sum += envelop;
        //if (checkPosture(envelop)) {
        //    badPostureCount++;
        //}
    }
    float averageEnvelope = sum / size;
    //float badPosturePercentage = (badPostureCount / (float)size) * 100.0;
    
    // Store bad posture percentage in bufferToSend
    //bufferToSend[3 * measurementUnitsBeforeSend + i] = badPosturePercentage;
    
    return averageEnvelope;
}

//EMG related functions
bool calibrateEMG() {
  const int calibrationTime = 10000; 
  int baselineValue = 1023; 
  int maxValue = 0;  
  unsigned long startTime = millis();
  while (millis() - startTime < calibrationTime) {
    int readValue = analogRead(EMG_Input);
        
    // Update baseline (minimum) and max values
    if (readValue < baselineValue) {
      baselineValue = readValue;
    }
    if (readValue > maxValue) {
      maxValue = readValue;
    }
        
    // Display progress
    Serial.print("Calibrating... Min: ");
    Serial.print(baselineValue);
    Serial.print(" Max: ");
    Serial.println(maxValue);
    delay(5);
  }
  int signalRange = maxValue - baselineValue;
  badPostureThreshold = baselineValue + (0.1 * signalRange); //is never actually used
  return true; //calibration is done
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


//not needed as is
bool checkPosture(int envelope) {
    if (envelope > 30) {
        return true; // Bad posture detected
    }
    return false; // Good posture
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
