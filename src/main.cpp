#include <SoftwareSerial.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

// Define the RX and TX pins for SoftwareSerial
#define RX_PIN 12
#define TX_PIN 13
#define LED 16

const char* ssid = "Tanand_Hardware";
const char* password = "202040406060808010102020";

static unsigned long lastPrintTime = 0; 
const unsigned long printInterval = 500; 

const char* mqtt_server = "192.168.0.110";  
WiFiClient espClient;
PubSubClient client(espClient);

// Initialize SoftwareSerial
SoftwareSerial debug(RX_PIN, TX_PIN);

char receivedData[6] = {0};
uint8_t dataIndex = 0;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  debug.print("Message arrived on topic: ");
  debug.print(topic);
  debug.print(". Message: ");
  for (int i = 0; i < length; i++) {
    debug.print((char)payload[i]);
  }
  debug.println();
}

void sendMQTTMessage(String message) {
  if (client.connected()) {
    client.publish("radar/data", message.c_str()); // Topic to publish the message
    // radar.println("Message forwarded to MQTT");
  } else {
    debug.println("MQTT not connected, retrying...");
    while (!client.connected()) {
      if (client.connect("LoRaGateway")) {
        debug.println("MQTT reconnected!");
      } else {
        debug.print("MQTT Connection failed, retrying in 5 seconds...");
        delay(5000);
      }
    }
  }
}

void connectToWiFi() {
  debug.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debug.print(".");
  }
  debug.println("Wi-Fi connected!");
}

void connectToMQTT() {
  while (!client.connected()) {
    if (client.connect("LoRaGateway")) {
      debug.println("MQTT Connected!");
    } else {
      debug.print("MQTT Connection failed, retrying in 5 seconds...");
      delay(5000);
    }
  }
}

String parseToJson(uint8_t presence, uint16_t distance) {
  // Create a JSON object
  JsonDocument doc;
  doc["presence"] = (presence == 0 || presence == 1) ? "No motion detected" : "Motion detected";
  doc["distance"] = distance;

  // Serialize the JSON object to a string
  String jsonMessage;
  serializeJson(doc, jsonMessage);

  return jsonMessage;
}

void setDetectionDistance(float minDistance, float maxDistance) {
    // Convert meters to radar units (1 unit = 0.7 m)
    uint8_t minUnits = (uint8_t)(minDistance / 0.7);
    uint8_t maxUnits = (uint8_t)(maxDistance / 0.7);

    // Ensure values are within valid range
    if (minUnits > 16) minUnits = 16;
    if (maxUnits > 16) maxUnits = 16;
    if (minUnits > maxUnits) {
        debug.println("Error: Minimum distance cannot be greater than maximum distance.");
        return;
    }

    // Build command to set detection distance
    byte setDistanceCommand[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x0E, 0x00, 0x70, 0x00,
        0x05, 0x00, maxUnits, 0x00, 0x00, 0x00, 0x0A, 0x00, minUnits, 0x00, 0x00, 0x00,
        0x04, 0x03, 0x02, 0x01
    };

    byte enableCommand[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    Serial.write(enableCommand, sizeof(enableCommand));
    delay(100);

    Serial.write(setDistanceCommand, sizeof(setDistanceCommand));
    delay(100);

    // End configuration mode
    byte endCommand[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    Serial.write(endCommand, sizeof(endCommand));
    delay(100);
}

void setAbsenceDelay(uint16_t delayTime) {
    // Ensure the delay time is within valid range
    if (delayTime < 10 || delayTime > 120) {
        debug.println("Error: Absence delay must be between 10 and 120 seconds.");
        return;
    }

    // Convert delay time to little-endian format
    byte lowByte = delayTime & 0xFF;
    byte highByte = (delayTime >> 8) & 0xFF;

    // Build command to set absence delay
    byte setAbsenceDelayCommand[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x70, 0x00,
        0x06, 0x00, lowByte, highByte, 0x00, 0x00,
        0x04, 0x03, 0x02, 0x01
    };

    // Send command to radar
    byte enableCommand[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    Serial.write(enableCommand, sizeof(enableCommand));
    delay(100);

    debug.write(setAbsenceDelayCommand, sizeof(setAbsenceDelayCommand));
    delay(100);

    byte endCommand[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
    Serial.write(endCommand, sizeof(endCommand));
    delay(100);

    debug.print("Set absence delay to: ");
    debug.print(delayTime);
    debug.println(" seconds.");
}

void setup() {
  Serial.begin(256000); // For debugging
  debug.begin(9600); // Default baud rate for RD-03L
  // pinMode(LED, OUTPUT); 
  connectToWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
  connectToMQTT();

  setDetectionDistance(0, 5);
  setAbsenceDelay(10);
  // debug.println("RD-03L Radar Module Initialized");
  delay(100);
}

void loop() {
  client.loop(); 

  while (Serial.available() > 0) {
    char c = Serial.read();
    delay(50);
    // debug.print("0x");
    // debug.println((uint8_t)c, HEX);

    if (dataIndex == 0 && (uint8_t)c == 0x6E) {
      receivedData[dataIndex++] = c;
    } else if (dataIndex > 0) {
      receivedData[dataIndex++] = c;

      if (dataIndex == 5) {
        // Validate packet
        if ((uint8_t)receivedData[4] == 0x62) {
          uint8_t presence = (uint8_t)receivedData[1];
          uint16_t distance = ((uint8_t)receivedData[3] << 8) | (uint8_t)receivedData[2];

          unsigned long currentTime = millis();
          if (currentTime - lastPrintTime >= printInterval) {
            lastPrintTime = currentTime;

            String presenceStatus;
            if (presence == 0 || presence == 1) {
              presenceStatus = "No motion detected.";
            } else if (presence == 2 || presence == 3) {
              presenceStatus = "Motion detected.";
            } else {
              presenceStatus = "Unknown status.";
            }

            debug.println("Presence Status: " + String(presenceStatus));
            debug.println("Distance: " +  String(distance) + " cm");

            String jsonMessage = parseToJson(presence, distance);
            sendMQTTMessage(jsonMessage);
          }
        } else {
          debug.println("Invalid packet detected.");
        }
        memset(receivedData, 0, sizeof(receivedData));
        dataIndex = 0;
      }
    }

    // Prevent buffer overflow
    if (dataIndex >= sizeof(receivedData)) {
      memset(receivedData, 0, sizeof(receivedData));
      dataIndex = 0;
    }
  }
  delay(2000);
} 