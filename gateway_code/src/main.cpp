#include <SPI.h>
#include <LoRa.h>
#include <Arduino.h>

const int aantalNodes = 10;  // MAX aantal nodes
uint8_t registeredNodes[aantalNodes];  // Array voor geregistreerde node ID's
uint8_t registeredCount = 0;           // Teller voor het aantal geregistreerde nodes
String interval = "2000"; // Tijd tussen de nodes

struct NodeData {
  uint8_t id;              // ID van de node
  uint8_t data[5];         // Data van de node (bijvoorbeeld 10, 20, 30, 40, 50)
};

NodeData nodeData[aantalNodes];  // Array om gegevens voor alle nodes op te slaan

void storeNodeData(uint8_t id, uint8_t* payload, size_t length) {
  for (uint8_t i = 0; i < registeredCount; i++) {
    if (nodeData[i].id == id) {
      // Kopieer de data naar de node's opslag
      memcpy(nodeData[i].data, payload, length);
      Serial.print("Data opgeslagen voor Node 0x");
      Serial.print(id, HEX);  // Print de node ID in hexadecimale vorm
      Serial.print(": ");

      // Print de opgeslagen data in hexadecimale vorm
      for (size_t j = 0; j < length; j++) {
        Serial.print("0x");
        if (payload[j] < 0x10) {
          Serial.print("0");  // Voeg een extra 0 toe voor eenheidscijfers
        }
        Serial.print(payload[j], HEX);  // Print de byte als hexadecimaal
        Serial.print(" ");
      }
      Serial.println();
      return;
    }
  }
  Serial.println("Error: Node niet gevonden voor data-opslag!");
}


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
void sendACK(uint8_t id, String interval, long remainingTime) {
  String data = String(id) + "," + String(interval) + "," + String(remainingTime);  // Voeg resterende tijd toe aan de string
  LoRa.beginPacket();
  LoRa.print(data);  // Stuur de string over LoRa
  LoRa.endPacket();

  Serial.print("Sending ACK to Node 0x");
  Serial.print(id, HEX);
  Serial.print(" with interval ");
  Serial.print(interval);
  Serial.print(" and remaining initialization time ");
  Serial.println(remainingTime);
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
  unsigned long duration = 30000;        // Initialisatietijd
  Serial.println("WE GAAN INITTTTS");

  while (millis() - startMillis < duration) {
    String response = "";
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // Verkrijg de inkomende gegevens als een string
      while (LoRa.available()) {
        response += (char)LoRa.read();
      }
      Serial.println("Ontvangen pakket: " + response);

      // Haal de node ID uit de ontvangen string (verondersteld dat het een hex-waarde is)
      uint8_t id = strtol(response.c_str(), NULL, 16);  // Zet de string om naar een hex waarde
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

        // Vul de `NodeData`-struct met het ID
        nodeData[registeredCount].id = id;

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
      long elapsedTime = (millis() - startMillis);  // Verstreken tijd in seconden
      long remainingTime = duration - elapsedTime;   // Resterende tijd
      Serial.println("Remaining time: " + String(remainingTime));
      sendACK(id, interval, remainingTime);  // Stuur ACK met interval en resterende tijd
    }
  }
  Serial.println("#########################################Initialisatie voltooid. Ga door naar de normale loop.#########################################");
}



void loop() {
  String response = "";
  while (true) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // Verkrijg de inkomende gegevens als een string
      while (LoRa.available()) {
        response += (char)LoRa.read();
      }
      Serial.println("Ontvangen pakket: " + response);
      break;  // Verlaat de while-loop zodra het pakket volledig is ontvangen
    }
  }

  // Split de response op basis van komma's
  int commaIndex = response.indexOf(',');
  if (commaIndex == -1) {
    Serial.println("Error: Ongeldig pakket ontvangen zonder komma.");
    return;  // Verlaat als er geen komma is
  }

  // Het eerste deel voor de ID
  String idStr = response.substring(0, commaIndex);  // Haal de ID uit het eerste deel
  uint8_t id = strtol(idStr.c_str(), NULL, 16);  // Zet de ID om naar uint8_t
  Serial.print("Ontvangen ID: 0x");
  Serial.println(id, HEX);

  // Controleer of de node geregistreerd is
  if (!isNodeRegistered(id)) {
    Serial.print("Ongeldige ID ontvangen: 0x");
    Serial.print(id, HEX);
    Serial.println(". Pakket genegeerd.");
    return;
  }

  // Haal de data op na de komma
  String dataStr = response.substring(commaIndex + 1);  // Alles na de eerste komma
  String dataArray[5];  // Array voor de data bytes
  int dataIndex = 0;

  // Splits de data op basis van komma's
  while (dataStr.length() > 0 && dataIndex < 5) {
    commaIndex = dataStr.indexOf(',');
    if (commaIndex == -1) {
      // Als er geen komma meer is, neem de rest van de string
      dataArray[dataIndex] = dataStr;
      break;
    }
    dataArray[dataIndex] = dataStr.substring(0, commaIndex);
    dataStr = dataStr.substring(commaIndex + 1);  // Verwijder de verwerkte data van de string
    dataIndex++;
  }

  // Zet de data om naar uint8_t
  uint8_t payload[5];
  for (int i = 0; i < 5; i++) {
    if (i < dataIndex) {
      payload[i] = strtol(dataArray[i].c_str(), NULL, 16);
      Serial.print("Data byte ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(payload[i], HEX);
    }
  }

  // Sla de data op bij de bijbehorende node
  storeNodeData(id, payload, 5);
}