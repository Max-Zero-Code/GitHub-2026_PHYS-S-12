#include "painlessMesh.h"
#include <ArduinoJson.h>
#include "DHT.h"

String nodeName;
static String deriveNodeName() {
#ifdef NODE_NAME_OVERRIDE
  return String(NODE_NAME_OVERRIDE);
#else
  uint64_t mac = ESP.getEfuseMac();
  char buf[20];
  snprintf(buf, sizeof(buf), "sensor-%02x%02x%02x",
           (uint8_t)(mac >> 16), (uint8_t)(mac >> 8), (uint8_t)mac);
  return String(buf);
#endif
}

#define MESH_SSID     "sensorMesh"
#define MESH_PASSWORD "meshpassword123"
#define MESH_PORT     5555
Scheduler    userScheduler;
painlessMesh mesh;
void sendReading();
Task taskSendReading(TASK_SECOND * 2, TASK_FOREVER, &sendReading);

#define IR_PIN   A0
#define GAS_PIN  A1
#define SOIL_PIN A2
#define DHTPIN   D4
#define DHTTYPE  DHT22
const float ADC_MAX_VOLTAGE = 3.3;
const int   ADC_RESOLUTION  = 4095;
const float GAS_DIVIDER_SCALE = 1.5;
DHT dht(DHTPIN, DHTTYPE);
unsigned long lastDHTRead = 0;
const unsigned long DHT_INTERVAL_MS = 2000;

int   latestIrRaw   = 0;
int   latestGasRaw  = 0;
int   latestSoilRaw = 0;
float latestTempC   = 0.0f;
float latestHum     = 0.0f;
bool  dhtValid      = false;
unsigned long lastDHTOkMs = 0;

void sendReading() {
  StaticJsonDocument<256> doc;
  doc["node"] = nodeName;
  doc["ir"]   = latestIrRaw;
  doc["gas"]  = latestGasRaw;
  doc["soil"] = latestSoilRaw;
  bool ok = dhtValid && (millis() - lastDHTOkMs < 10000);
  doc["temp"]  = ok ? latestTempC : 0.0f;
  doc["hum"]   = ok ? latestHum   : 0.0f;
  doc["dhtok"] = ok;
  String payload;
  serializeJson(doc, payload);
  mesh.sendBroadcast(payload);
  Serial.print("Sent: ");
  Serial.println(payload);
}

void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("Relay/other node msg from %u: %s\n", from, msg.c_str());
}
void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New mesh connection: %u\n", nodeId);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  nodeName = deriveNodeName();
  analogReadResolution(12);
  dht.begin();
  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  userScheduler.addTask(taskSendReading);
  taskSendReading.enable();
  Serial.println();
  Serial.println(F("=== ALL SENSORS TEST ==="));
  Serial.println(F("IR (A0) | Gas (A1) | Soil (A2) | DHT22 (D4)"));
  Serial.println();
  Serial.printf("%s ready, joining mesh.\n", nodeName.c_str());
}

void loop() {
  mesh.update();
  static unsigned long lastSample = 0;
  if (millis() - lastSample < 300) return;
  lastSample = millis();
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
  if (millis() - lastDHTRead >= DHT_INTERVAL_MS) {
    lastDHTRead = millis();
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    Serial.print(F("  |  DHT: "));
    if (isnan(h) || isnan(t)) {
      Serial.print(F("read failed"));
      dhtValid = false;
    } else {
      Serial.print(t, 1);
      Serial.print(F("C "));
      Serial.print(h, 1);
      Serial.print(F("%"));
      latestTempC = t;
      latestHum   = h;
      dhtValid    = true;
      lastDHTOkMs = millis();
    }
  }
  Serial.println();
  latestIrRaw   = irRaw;
  latestGasRaw  = gasRaw;
  latestSoilRaw = soilRaw;
}