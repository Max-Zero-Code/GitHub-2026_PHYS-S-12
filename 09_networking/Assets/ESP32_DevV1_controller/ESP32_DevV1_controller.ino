/*
  All-sensors test — IR phototransistor, MQ-5 gas, soil moisture, AM2302/DHT22
  XIAO ESP32C3 — verified pin map

  Wiring:
    IR phototransistor -> A0 (GPIO2)
    MQ-5 gas sensor     -> A1 (GPIO3), 10k series + 20k to GND divider
    Soil moisture       -> A2 (GPIO4)
    AM2302 / DHT22 data -> D3 (GPIO5), 10k pull-up to 3V3
                           (DHT VCC can be 5V - pull-up still goes to 3V3,
                           see prior notes on why)

  Install via Library Manager: "DHT sensor library" by Adafruit,
  plus its dependency "Adafruit Unified Sensor".
*/

#include "DHT.h"

#define IR_PIN   D0
#define GAS_PIN  D1
#define SOIL_PIN D2
#define DHTPIN   D4
#define DHTTYPE  DHT22

const float ADC_MAX_VOLTAGE = 3.3;
const int   ADC_RESOLUTION  = 4095;
const float GAS_DIVIDER_SCALE = 1.5;   // (10k + 20k) / 20k

DHT dht(DHTPIN, DHTTYPE);

unsigned long lastDHTRead = 0;
const unsigned long DHT_INTERVAL_MS = 2000;   // DHT22 min interval between reads

void setup() {
  Serial.begin(115200);
  delay(500);

  analogReadResolution(12);
  dht.begin();

  Serial.println();
  Serial.println(F("=== ALL SENSORS TEST ==="));
  Serial.println(F("IR (A0) | Gas (A1) | Soil (A2) | DHT22 (D3)"));
  Serial.println();
}

void loop() {
  int irRaw = analogRead(IR_PIN);
  float irVolts = irRaw * (ADC_MAX_VOLTAGE / ADC_RESOLUTION);

  int gasRaw = analogRead(GAS_PIN);
  float gasPinVolts = gasRaw * (ADC_MAX_VOLTAGE / ADC_RESOLUTION);
  float gasSensorVolts = gasPinVolts * GAS_DIVIDER_SCALE;

  int soilRaw = analogRead(SOIL_PIN);
  float soilVolts = soilRaw * (ADC_MAX_VOLTAGE / ADC_RESOLUTION);

  Serial.print(F("IR raw: "));
  Serial.print(irRaw);
  Serial.print(F(" ("));
  Serial.print(irVolts, 2);
  Serial.print(F("V)  |  Gas raw: "));
  Serial.print(gasRaw);
  Serial.print(F(" ("));
  Serial.print(gasSensorVolts, 2);
  Serial.print(F("V)  |  Soil raw: "));
  Serial.print(soilRaw);
  Serial.print(F(" ("));
  Serial.print(soilVolts, 2);
  Serial.print(F("V)"));

  // DHT22 only every 2s - reading more often than that returns stale/failed data
  if (millis() - lastDHTRead >= DHT_INTERVAL_MS) {
    lastDHTRead = millis();
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    Serial.print(F("  |  DHT: "));
    if (isnan(h) || isnan(t)) {
      Serial.print(F("read failed"));
    } else {
      Serial.print(t, 1);
      Serial.print(F("C "));
      Serial.print(h, 1);
      Serial.print(F("%"));
    }
  }

  Serial.println();
  delay(300);
}