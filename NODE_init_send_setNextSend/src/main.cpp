#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoLowPower.h>

#define ss 5
#define rst 14
#define dio0 2

// put function declarations here:
int counter = 0;
int Errorled = 6;
int myID = 0;
int sendInterval = 0;
int dataArray[] = {10, 20, 30, 40, 50};
int arraySize = sizeof(dataArray) / sizeof(dataArray[0]);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");
  LoRa.setPins(ss, rst, dio0);
  pinMode(Errorled, OUTPUT);
  setErrorLEDLow();

  while (!LoRa.begin(866E6)) {
    Serial.println("Starting LoRa failed!");
    ErrorLoRaBegin();
  }


  //dit afwerken, moet nog receiven van gate bij setup
  sendInterval = initWithGate();

  //--------------these might be the so called TAGS for the nodes----------------
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  //LoRa.setSyncWord(0xF3);
  //Serial.println("LoRa Initializing OK!");
//--------------------------------------------------------------------------------


}

void loop() {
  //loop nog volledig doen, moet telkens om de zoveel tijd zenden, en nieuwe tijd ontvangen van gate
  LowPower.deepSleep(sendInterval);
  
  loopSendMeasurements();
  sendInterval = loopSynchronize();
}


// send NODE ID to gate
int initWithGate(){
  initWithGateAuthenticate();
  return initWithGateSynchronize();
}

void initWithGateAuthenticate(){
  digitalWrite(Errorled, LOW);
  LoRa.beginPacket();
  LoRa.write(myID);
  LoRa.endPacket();
}


//to receive initial interval to send again
//possibly add a check to see if the data comes from the gate
int initWithGateSynchronize(){
  int interval = 0;
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    
    //////////////////////////
    //this is probably the best way to read the data as we expect an integer
    interval = LoRa.parseInt();
    Serial.println("Received interval: " + interval);
    //or maybe like this
    //while (LoRa.available()) {
    //  String LoRaData = LoRa.readString();
    //  Serial.print(LoRaData); 
    //}

    //////////////////////////
  }
  return interval;
}




void loopSendMeasurements(){
  //loop nog volledig doen, moet telkens om de zoveel tijd zenden, en nieuwe tijd ontvangen van gate
  LowPower.deepSleep(sendInterval);
  
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
  for (int i = 0; i < arraySize; i++) {
    LoRa.write(dataArray[i]);
  }
  LoRa.endPacket();
}

int loopSynchronize(){
  int interval = 0;
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    
    //////////////////////////
    //this is probably the best way to read the data as we expect an integer
    interval = LoRa.parseInt();
    Serial.println("Received interval: " + interval);
    //or maybe like this
    //while (LoRa.available()) {
    //  String LoRaData = LoRa.readString();
    //  Serial.print(LoRaData); 
    //}

    //////////////////////////
  }
  return interval;
}




//LED ERROR SIGNS


//incase LoRa.begin() fails, LED will stay high as long as it fails
void ErrorLoRaBegin(){
  digitalWrite(Errorled, HIGH);
  delay(2000);
}

void setErrorLEDLow(){
  digitalWrite(Errorled, LOW);
}

//OR GET NODE ID FROM GATE


