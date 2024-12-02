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

const char* mqtt_server = "192.168.0.109";  
WiFiClient espClient;
PubSubClient client(espClient);

// Initialize SoftwareSerial
SoftwareSerial radar(RX_PIN, TX_PIN);
static unsigned long lastReadTime = 0;
const unsigned long readInterval = 100; // Set a delay of 500 ms between readings

char receivedData[6] = {0};
uint8_t dataIndex = 0;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  radar.print("Message arrived on topic: ");
  radar.print(topic);
  radar.print(". Message: ");
  for (int i = 0; i < length; i++) {
    radar.print((char)payload[i]);
  }
  radar.println();
}

void sendMQTTMessage(String message) {
  if (client.connected()) {
    client.publish("radar/data", message.c_str()); // Topic to publish the message
    // radar.println("Message forwarded to MQTT");
  } else {
    radar.println("MQTT not connected, retrying...");
    while (!client.connected()) {
      if (client.connect("LoRaGateway")) {
        radar.println("MQTT reconnected!");
      } else {
        radar.print("MQTT Connection failed, retrying in 5 seconds...");
        delay(5000);
      }
    }
  }
}

void connectToWiFi() {
  radar.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    radar.print(".");
  }
  radar.println("Wi-Fi connected!");
}

void connectToMQTT() {
  while (!client.connected()) {
    if (client.connect("LoRaGateway")) {
      radar.println("MQTT Connected!");
    } else {
      radar.print("MQTT Connection failed, retrying in 5 seconds...");
      delay(5000);
    }
  }
}

String parseToJson(uint8_t presence, uint16_t distance) {
  // Create a JSON object
  StaticJsonDocument<200> doc;
  doc["presence"] = (presence == 0 || presence == 1) ? "No motion detected" : "Motion detected";
  doc["distance"] = distance;

  // Serialize the JSON object to a string
  String jsonMessage;
  serializeJson(doc, jsonMessage);

  return jsonMessage;
}

void setup() {
  Serial.begin(256000); // For debugging
  radar.begin(9600); // Default baud rate for RD-03L
  pinMode(LED, OUTPUT); 
  connectToWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
  connectToMQTT();

  // radarModule.begin(Serial, RX_PIN, TX_PIN, 256);
  radar.println("RD-03L Radar Module Initialized");
  delay(100);

}

void loop() {
  client.loop(); 
  
  // digitalWrite(LED, HIGH);    
  // delay(2000);                     
  // digitalWrite(LED, LOW);   
  // delay(2000);  

  // radar.println("Hi");
  while (Serial.available() > 0) {
    char c = Serial.read();
    delay(50);
    radar.print("0x");
    radar.println((uint8_t)c, HEX);

    if (dataIndex == 0 && (uint8_t)c == 0x6E) {
      receivedData[dataIndex++] = c;
    } else if (dataIndex > 0) {
      receivedData[dataIndex++] = c;

      if (dataIndex == 5) {
        // Validate packet
        if ((uint8_t)receivedData[4] == 0x62) {
          uint8_t presence = (uint8_t)receivedData[1];
          uint16_t distance = ((uint8_t)receivedData[3] << 8) | (uint8_t)receivedData[2];

        String presenceStatus;
        if (presence == 0 || presence == 1) {
          presenceStatus = "No motion detected.";
        } else if (presence == 2 || presence == 3) {
          presenceStatus = "Motion detected.";
        } else {
          presenceStatus = "Unknown status.";
        }

        // Print the decoded information
        radar.println("Presence Status: " + String(presenceStatus));
        radar.println("Distance: " +  String(distance) + " cm");
        String jsonMessage = parseToJson(presence, distance);
        sendMQTTMessage(jsonMessage);
        } else {
          Serial.println("Invalid packet detected.");
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

  // Small delay for stability
  delay(10);
} 