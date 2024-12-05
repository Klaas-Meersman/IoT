#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"



MAX30105 particleSensor;

int i = 0;
const int averBufferSize = 10;
uint32_t averageBuffer[averBufferSize] = { 0 };


const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
const int setYaxis = 100;


void setup() {
  Serial.begin(115200);
  Wire.begin();


  if (particleSensor.begin() == false) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1)
      ;
  }

  particleSensor.setup();  //Configure sensor. Use 6.4mA for LED drive
}

void loop() {
  //float temperature = sensor.readTempC();
  //float temperature2 = particleSensor.readTemperature();


  int32_t average;
  int32_t irValue = particleSensor.getIR();
  averageBuffer[i] = irValue;
  int heartbeat = heartBeat(irValue);
  i++;
  i = i % averBufferSize;
  int32_t sum = 0;
  for (int j = 0; j < averBufferSize; j++) {
    sum += averageBuffer[j];
    //Serial.println(averageBuffer[j]);
  }
  //Serial.println(sum);
  average = sum / averBufferSize;

  //  Serial.print("Temperature: ");
  //  Serial.print(temperature-2);
  //  Serial.println(" °C");
  //  Serial.print("Temperature2=");
  //  Serial.println(temperature2-4.5);
  // Wait for 1 second before next reading


  Serial.print(setYaxis);
  Serial.print(", ");
  Serial.print(irValue - average);
  Serial.print(", ");
  Serial.print(heartbeat);
  Serial.print(", ");
  Serial.println(-setYaxis);
}


int heartBeat(int32_t irValue) {
  if (irValue > 80000) {  // If a finger is detected

    if (checkForBeat(irValue) == true)  // If a heart beat is detected
    {

      //Serial.println("Heartbeat waargenomen; IR value: "+ irValue);

      // We sensed a beat!
      long delta = millis() - lastBeat;  // Measure duration between two beats
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);  // Calculating the BPM


      if (beatsPerMinute < 255 && beatsPerMinute > 20)  // To calculate the average we strore some values (4) then do some math to calculate the average
      {
        rates[rateSpot++] = (byte)beatsPerMinute;  // Store this reading in the array
        rateSpot %= RATE_SIZE;                     // Wrap variable

        // Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
        return setYaxis;
      }
    }
  }
  return 0;
  if (irValue < 80000) {  // If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
    //Serial.println("Zit in irValue onder 500: " + irValue);
    beatAvg = 0;
  }
}
