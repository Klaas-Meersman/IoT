#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>

const int aantalNodes = 10;  // MAX aantal nodes
const int totalTime = 600;   // Totaal tijdsbereik in seconden (10 minuten = 600 seconden)
uint8_t registeredNodes[aantalNodes];  // Array voor geregistreerde node ID's
uint8_t registeredCount = 0;           // Teller voor het aantal geregistreerde nodes

void loopInitialize();

// Functie om te controleren of een node al is geregistreerd
bool isNodeRegistered(uint8_t id) {
  for (uint8_t i = 0; i < registeredCount; i++) {
    if (registeredNodes[i] == id) {
      return true;  // Node is al geregistreerd
    }
  }
  return false;  // Node is nog niet geregistreerd
}

// Functie om een ACK te verzenden
void sendACK(uint8_t id, int interval) {
  String data = String(id) + "," + String(interval);  // Maak een string van id en interval
  Serial.print(data);
  LoRa.beginPacket();
  LoRa.print(data);  // Stuur de string over LoRa
  LoRa.endPacket();

  Serial.print("Sending ACK to Node 0x");
  Serial.print(id, HEX);
  Serial.print(" with interval ");
  Serial.println(interval);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wacht tot de seriële poort klaar is
  Serial.println("Gateway klaar om nodes te registreren");
  LoRa.begin(868E6);  // Stel de frequentie in
  loopInitialize();
}

void loopInitialize() {
  unsigned long startMillis = millis();  // Begin de timer
  unsigned long duration = 240000;  // 4 minuten in milliseconden (4 * 60 * 1000)
  Serial.println("WE GAAN INITTTTS");

  while (millis() - startMillis < duration) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      uint8_t id = LoRa.read();  // Lees de ontvangen byte als ID
      Serial.print("Received ID: 0x");
      Serial.println(id, HEX);

      // Controleer of de node al geregistreerd is
      if (!isNodeRegistered(id)) {
        if (registeredCount >= aantalNodes) {
          Serial.println("Error: Maximum aantal nodes bereikt!");
          return;
        }
        // Registreer de node door de ID toe te voegen aan de array
        registeredNodes[registeredCount] = id;
        registeredCount++;

        Serial.print("Node 0x");
        Serial.print(id, HEX);
        Serial.println(" geregistreerd.");
      } else {
        Serial.print("Node 0x");
        Serial.print(id, HEX);
        Serial.println(" is al geregistreerd.");
      }

      // Bereken het interval voor de node op basis van het aantal nodes
      int interval = 1000;  // Tijdsinterval (in dit geval hardcoded)
      sendACK(id, interval);  // Stuur ACK met het interval
    }
  }

  Serial.println("Initialisatie voltooid. Ga door naar de normale loop.");
}

void loop(){
  Serial.println("JAJA INITIALIZE IS GEDAAN NU IS HET TIJD VOOR DATA MOOFFUUCKER (Amerikaans accent gebruiken voor het uitschelden)");
}
