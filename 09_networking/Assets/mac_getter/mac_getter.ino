#include "WiFi.h"

void setup() {
  Serial.begin(115200);
  delay(2000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  // Fallback: read from eFuse directly as uint64
  uint64_t chipid = ESP.getEfuseMac();
  Serial.printf("Chip MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
    (uint8_t)(chipid >> 40),
    (uint8_t)(chipid >> 32),
    (uint8_t)(chipid >> 24),
    (uint8_t)(chipid >> 16),
    (uint8_t)(chipid >> 8),
    (uint8_t)(chipid));
}

void loop() {}
