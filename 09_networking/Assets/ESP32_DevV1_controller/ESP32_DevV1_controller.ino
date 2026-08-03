/*
  Sensor node - reads IR, gas, soil, DHT22 and sends over ESP-NOW
  XIAO ESP32-C3 (the sensor board)

  Wiring:
    IR phototransistor -> D0
    MQ-5 gas sensor     -> D1
    Soil moisture        -> D2
    AM2302 / DHT22 data -> D4, 10k pull-up to 3V3

  Install: "DHT sensor library" + "Adafruit Unified Sensor" (Library Manager)
*/

#include <WiFi.h>
#include <esp_now.h>
#include "DHT.h"

#define IR_PIN   D0
#define GAS_PIN  D1
#define SOIL_PIN D2
#define DHTPIN   D4
#define DHTTYPE  DHT22

DHT dht(DHTPIN, DHTTYPE);

// Must match the struct in the receiver sketch exactly
typedef struct SensorPacket {
  int   irRaw;
  int   gasRaw;
  int   soilRaw;
  float tempC;
  float humidity;
  bool  dhtValid;
} SensorPacket;

SensorPacket packet;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
  dht.begin();

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Sensor node ready, broadcasting via ESP-NOW.");
}

void loop() {
  packet.irRaw   = analogRead(IR_PIN);
  packet.gasRaw  = analogRead(GAS_PIN);
  packet.soilRaw = analogRead(SOIL_PIN);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    packet.dhtValid = false;
  } else {
    packet.dhtValid = true;
    packet.tempC = t;
    packet.humidity = h;
  }

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet));

  Serial.print("IR: "); Serial.print(packet.irRaw);
  Serial.print("  Gas: "); Serial.print(packet.gasRaw);
  Serial.print("  Soil: "); Serial.print(packet.soilRaw);
  Serial.print("  DHT: ");
  if (packet.dhtValid) {
    Serial.print(packet.tempC); Serial.print("C "); Serial.print(packet.humidity); Serial.print("%");
  } else {
    Serial.print("invalid");
  }
  Serial.println(result == ESP_OK ? "  [sent]" : "  [send error]");

  delay(2000);   // DHT22 minimum interval
}