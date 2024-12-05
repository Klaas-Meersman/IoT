#include <Arduino.h>
#include <ArduinoLowPower.h>

#include <Wire.h>
#include "MAX30105.h"  //MAX3010x library
#include "heartRate.h" //Heart rate calculating algorithm


MAX30105 heartRateSensor;
const int GSR = A0; //analog pin 0


//-----------CHANGE THESE PARAMETERS-------------
const byte amountOfHeartRateMeasurementsPerUnit = 5;
const byte amountOfSkinConductanceMeasurementsPerUnit = 10;
const byte amountOfSkinTempMeasurementsPerUnit = 10;
const int deltaMeasurementUnitsInSec = 60; // 60 seconds
const int deltaSendToGateInMinutes = 5; // 5 minutes
const int amountOfSensors = 3;
//-----------------------------------------------




//dont change these parameters, they are calculated based on the previous ones
const int deltaSendToGateInMilis = deltaSendToGateInMinutes*60000;
const int deltaMeasurementUnitsInMilis = deltaMeasurementUnitsInSec*1000;
const byte measurementUnitsBeforeSend = int(deltaSendToGateInMilis/deltaMeasurementUnitsInMilis);
float bufferToSend[measurementUnitsBeforeSend * amountOfSensors];

float measurementUnitHeartRateSensor(byte);
float measurementUnitSkinConductanceSensor(byte);
float measurementUnitSkinTemperature(byte);
void performMeasurements();

void setup() {
  heartRateSensor.begin(Wire, I2C_SPEED_FAST); // Use default I2C port, 400kHz speed
  heartRateSensor.setup();                     // Configure sensor with default settings
  heartRateSensor.setPulseAmplitudeRed(0x0A);  // Turn Red LED to low to indicate sensor is running

}

void loop() {
  //note that the bufferToSend is a float array, so we can store the heart rate and the skin conductance in the same array, thas why we multiply by 2
  //the buffer first stores #measurementUnitsBeforeSend heart rate measurements, then #measurementUnitsBeforeSend skin conductance measurements
  performMeasurementsWithSleepInBetween();


  sendData();
  LowPower.deepSleep(deltaSendToGateInMilis); //sleep for .. minutes
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

//-- no need to copy this
void sendData(){
   //---send data in bufferToSend to the gateway
  for(int i = 0; i < amountOfSensors*measurementUnitsBeforeSend; i++){
    Serial.println(bufferToSend[i]);
  }
  //send data to the gateway---

}
//-- no need to copy this



//we can get stuck on this becuz no heartrate is measured (can be finicky)
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

float measurementUnitSkinConductanceSensor(byte size){
  int sum;
  for(int i = 0;i<size;i++){
    sum += analogRead(GSR);
  }
  return sum/size;
}

float measurementUnitSkinTemperature(byte size){
  int sum;
  for(int i = 0; i<size;i++){
    sum += tempSensor.readTempC();
  }
  return sum/size;
}