#include <Arduino.h>
//#include <SPI.h>
#include <LoRa.h>
//#include <ArduinoLowPower.h>

//#define ss 5
//#define rst 14
//#define dio0 2

// put function declarations here:
//int Errorled = 6;

uint8_t myID = 0xA0;

int sendInterval = 0;
int dataArray[] = {10, 20, 30, 40, 50};
int arraySize = sizeof(dataArray) / sizeof(dataArray[0]);


void initWithGateAuthenticate() {
  LoRa.beginPacket();
  LoRa.write(myID);
  LoRa.endPacket();
  Serial.println("Sending done");
}


int waitForResponse() {
  String response = "";
  //Serial.println("Aan het wachten op een antwoord van de Gateway...");
  
  uint8_t ACKID = 0x00;
  int interval = -1;

  // Wacht onbeperkt op een bericht
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


  if (response.length() > 0) {
    int commaIndex = response.indexOf(',');
    if (commaIndex != -1) {
      uint8_t id = (uint8_t)response.substring(0, commaIndex).toInt();
      if(id==myID){
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
  return waitForResponse();
}



void loopSendMeasurements(){
  //loop nog volledig doen, moet telkens om de zoveel tijd zenden, en nieuwe tijd ontvangen van gate
  
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

//LED ERROR SIGNS
//incase LoRa.begin() fails, LED will stay high as long as it fails
void ErrorLoRaBegin(){
  //digitalWrite(Errorled, HIGH);
  delay(2000);
}

void setErrorLEDLow(){
  //digitalWrite(Errorled, LOW);
}

//OR GET NODE ID FROM GATE

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("LoRa Sender");
  //LoRa.setPins(ss, rst, dio0);
  //pinMode(Errorled, OUTPUT);
  //setErrorLEDLow();

  while (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    ErrorLoRaBegin();
  }

  sendInterval = initWithGate();
  //Serial.println("Intialization complete, send interval is:" + sendInterval);



  //--------------these might be the so called TAGS for the nodes----------------
  //Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
//--------------------------------------------------------------------------------

}

void loop() {
  //LowPower.deepSleep(sendInterval);
  delay(sendInterval);
  Serial.println("I sleep for" + sendInterval);
  Serial.println(".");
  //loopSendMeasurements();
}
















