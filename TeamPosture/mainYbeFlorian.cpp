#include <Arduino.h> 

#define SAMPLE_RATE 500        // Sampling frequency in Hz
#define PRINT_RATE 10          // Frequency of prints in Hz
#define BAUD_RATE 9600
#define EMG_Input A0
#define BUFFER_SIZE 128

int circular_buffer[BUFFER_SIZE];
int data_index, sum;
int print_counter = 0;
int baselineValue = 1023; 
int maxValue = 0;         
const int calibrationTime = 10000;  
bool calibrated = false;  
int badPostureThreshold;

float EMGFilter(float input)
{
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
	sum -= circular_buffer[data_index];
	sum += abs_emg;
	circular_buffer[data_index] = abs_emg;
	data_index = (data_index + 1) % BUFFER_SIZE;
	return (sum/BUFFER_SIZE) * 2;
}

void calibrateSensor() {
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
    badPostureThreshold = baselineValue + (0.1 * signalRange);
}

bool checkPosture(int envelope) {
    if (envelope > 30) {
        return true; // Bad posture detected
    }
    return false; // Good posture
}

void setup() {
	// Serial connection begin
	Serial.begin(BAUD_RATE);
  pinMode(EMG_Input, INPUT);
  calibrateSensor();
}

void loop() {
	// Calculate elapsed time
	static unsigned long past = 0;
	unsigned long present = micros();
	unsigned long interval = present - past;
	past = present;

	// Run timer
	static long timer = 0;
	timer -= interval;

	// Sample and get envelop
	if(timer < 0) {
		timer += 1000000 / SAMPLE_RATE;
		int sensor_value = analogRead(EMG_Input);
		int signal = EMGFilter(sensor_value);
		int envelop = getEnvelop(abs(signal));

    // Print at a lower rate
    print_counter++;
    if (checkPosture(envelop)) {
      Serial.println("Warning: Bad posture detected!");
    }

    if(print_counter >= SAMPLE_RATE / PRINT_RATE) {
      Serial.print(">");
      Serial.print("var1:");
      Serial.print(signal);
      Serial.print(",");
      Serial.print("var2:");
      Serial.println(envelop);
      print_counter = 0;
    }

	}
}