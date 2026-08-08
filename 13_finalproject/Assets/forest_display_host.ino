#include <Arduino.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <SPI.h>
#include "painlessMesh.h"
#include <ArduinoJson.h>

#define MESH_SSID     "sensorMesh"
#define MESH_PASSWORD "meshpassword123"   
#define MESH_PORT     5555
#define MAX_NODES     6
#define STALE_MS      10000UL   
struct NodeReading {
  String        name;
  int           ir, gas, soil;
  float         temp, hum;
  bool          dhtOk;        
  unsigned long lastSeenMs;
  bool          everSeen;
};
NodeReading nodes[MAX_NODES];
int  knownNodeCount = 0;
bool nodesDirty     = true;   
Scheduler   userScheduler;
painlessMesh mesh;
int findOrAddNode(const String &name) {
  for (int i = 0; i < knownNodeCount; i++) {
    if (nodes[i].name == name) return i;
  }
  if (knownNodeCount < MAX_NODES) {
    nodes[knownNodeCount].name = name;
    nodes[knownNodeCount].everSeen = false;
    return knownNodeCount++;
  }
  return -1;
}
bool nodeIsStale(const NodeReading &n) {
  return !n.everSeen || (millis() - n.lastSeenMs > STALE_MS);
}

class LGFX : public lgfx::LGFX_Device {
  lgfx::Bus_SPI       _bus_instance;
  lgfx::Panel_ILI9488 _panel_instance;
  
public:
  LGFX(void) {
    auto bus_cfg = _bus_instance.config();
    bus_cfg.spi_host   = SPI2_HOST;
    bus_cfg.spi_mode   = 0;
    bus_cfg.freq_write = 8000000;
    bus_cfg.freq_read  = 4000000;
    bus_cfg.pin_sclk   = D4;
    bus_cfg.pin_mosi   = D10;
    bus_cfg.pin_miso   = D3;   
    bus_cfg.pin_dc     = D2;
    bus_cfg.spi_3wire  = false;
    bus_cfg.use_lock   = true;
    _bus_instance.config(bus_cfg);
    _panel_instance.setBus(&_bus_instance);
    auto panel_cfg = _panel_instance.config();
    panel_cfg.pin_cs       = D1;
    panel_cfg.pin_rst      = D0;   
    panel_cfg.pin_busy     = -1;
    panel_cfg.panel_width  = 320;
    panel_cfg.panel_height = 480;
    panel_cfg.offset_rotation = 0;
    panel_cfg.bus_shared   = true;
    _panel_instance.config(panel_cfg);
    setPanel(&_panel_instance);
  }
};
LGFX tft;

#define TOUCH_CS    D5
#define TOUCH_IRQ   D8
#define TOUCH_MISO  D3
#define TOUCH_CMD_X 0xD0
#define TOUCH_CMD_Y 0x90
#define TOUCH_FREQ  1000000
#define CAL_INSET   60
#define CAL_POINTS  5
#define RAW_MIN     100
#define RAW_MAX     4000

struct Calibration { float ax, bx, cx, ay, by, cy; };
Calibration cal = { 0.137f, 0.0f, -41.0f, 0.0f, 0.091f, -27.0f };

#define SKIP_CALIBRATION 0
uint16_t readTouchChannel(uint8_t cmd) {
  digitalWrite(TOUCH_CS, LOW);
  SPI.beginTransaction(SPISettings(TOUCH_FREQ, MSBFIRST, SPI_MODE0));
  SPI.transfer(cmd);
  uint8_t hi = SPI.transfer(0x00);
  uint8_t lo = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(TOUCH_CS, HIGH);
  return ((uint16_t)hi << 8 | lo) >> 3;
}
static uint16_t median5(uint16_t *v) {
  for (int i = 1; i < 5; i++) {
    uint16_t k = v[i];
    int j = i - 1;
    while (j >= 0 && v[j] > k) { v[j + 1] = v[j]; j--; }
    v[j + 1] = k;
  }
  return v[2];
}
bool readTouchRaw(uint16_t *rawX, uint16_t *rawY) {
  if (digitalRead(TOUCH_IRQ) != LOW) return false;   
  uint16_t xs[5], ys[5];
  for (int i = 0; i < 5; i++) {
    xs[i] = readTouchChannel(TOUCH_CMD_X);
    ys[i] = readTouchChannel(TOUCH_CMD_Y);
  }
  uint16_t x = median5(xs), y = median5(ys);
  *rawX = x;
  *rawY = y;
  return !(x < RAW_MIN || x > RAW_MAX || y < RAW_MIN || y > RAW_MAX);
}
bool readTouchPoint(int32_t *sx, int32_t *sy) {
  uint16_t rawX, rawY;
  if (!readTouchRaw(&rawX, &rawY)) return false;
  
  
  *sx = constrain((int32_t)lroundf(cal.ax * rawX + cal.bx * rawY + cal.cx), 0, tft.width()  - 1);
  *sy = constrain((int32_t)lroundf(cal.ay * rawX + cal.by * rawY + cal.cy), 0, tft.height() - 1);
  return true;
}
bool getTap(int32_t *tapX, int32_t *tapY) {
  static bool wasDown = false;
  int32_t x, y;
  if (readTouchPoint(&x, &y)) {
    if (!wasDown) { wasDown = true; *tapX = x; *tapY = y; return true; }
  } else if (digitalRead(TOUCH_IRQ) != LOW) {
    wasDown = false;
  }
  return false;
}

static void waitForStableTouch(uint16_t *rawX, uint16_t *rawY) {
  while (true) {
    uint16_t x1, y1, x2, y2;
    if (readTouchRaw(&x1, &y1)) {
      delay(40);
      if (readTouchRaw(&x2, &y2)
          && abs((int)x1 - (int)x2) < 80 && abs((int)y1 - (int)y2) < 80) {
        *rawX = x2; *rawY = y2;
        return;
      }
    }
    delay(10);
  }
}
static void waitForRelease(void) {
  while (digitalRead(TOUCH_IRQ) == LOW) delay(10);
  delay(200);
}
static void captureTarget(uint16_t *rawX, uint16_t *rawY) {
  uint16_t sx, sy;
  waitForStableTouch(&sx, &sy);
  uint32_t accX = sx, accY = sy;
  uint16_t n = 1;
  for (int i = 0; i < 14; i++) {
    uint16_t x, y;
    if (readTouchRaw(&x, &y)) { accX += x; accY += y; n++; }
    delay(10);
  }
  *rawX = accX / n; *rawY = accY / n;
}
static bool solve3x3(double m[3][3], const double v[3], double out[3]) {
  #define DET3(a) ( a[0][0]*(a[1][1]*a[2][2] - a[1][2]*a[2][1]) \
                  - a[0][1]*(a[1][0]*a[2][2] - a[1][2]*a[2][0]) \
                  + a[0][2]*(a[1][0]*a[2][1] - a[1][1]*a[2][0]) )
  double det = DET3(m);
  if (fabs(det) < 1e-6) return false;
  for (int c = 0; c < 3; c++) {
    double t[3][3];
    memcpy(t, m, sizeof(t));
    for (int r = 0; r < 3; r++) t[r][c] = v[r];
    out[c] = DET3(t) / det;
  }
  return true;
  #undef DET3
}
static void drawTarget(int32_t x, int32_t y) {
  tft.drawFastHLine(x - 14, y, 29, TFT_YELLOW);
  tft.drawFastVLine(x, y - 14, 29, TFT_YELLOW);
  tft.drawCircle(x, y, 7, TFT_YELLOW);
}

void calibrate(void) {
  const int32_t W = tft.width(), H = tft.height();
  const int32_t px[CAL_POINTS] = { CAL_INSET, W - CAL_INSET, W - CAL_INSET, CAL_INSET, W / 2 };
  const int32_t py[CAL_POINTS] = { CAL_INSET, CAL_INSET, H - CAL_INSET, H - CAL_INSET, H / 2 };
  uint16_t rx[CAL_POINTS], ry[CAL_POINTS];
  for (int i = 0; i < CAL_POINTS; i++) {
    tft.fillScreen(TFT_BLACK);
    drawTarget(px[i], py[i]);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    char msg[40];
    snprintf(msg, sizeof(msg), "Tap the target  (%d/%d)", i + 1, CAL_POINTS);
    tft.drawCenterString(msg, W / 2, H / 2 + 40);
    Serial.printf("calibration %d/%d: target at (%ld, %ld)\n",
                  i + 1, CAL_POINTS, (long)px[i], (long)py[i]);
    captureTarget(&rx[i], &ry[i]);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawCenterString("Got it - lift off", W / 2, H / 2 - tft.fontHeight() / 2);
    waitForRelease();
    Serial.printf("  captured raw(%u,%u)\n", rx[i], ry[i]);
  }
  double m[3][3] = {{0}}, vX[3] = {0}, vY[3] = {0};
  for (int i = 0; i < CAL_POINTS; i++) {
    double x = rx[i], y = ry[i];
    m[0][0] += x*x;  m[0][1] += x*y;  m[0][2] += x;
    m[1][0] += x*y;  m[1][1] += y*y;  m[1][2] += y;
    m[2][0] += x;    m[2][1] += y;    m[2][2] += 1.0;
    vX[0] += x*px[i];  vX[1] += y*px[i];  vX[2] += px[i];
    vY[0] += x*py[i];  vY[1] += y*py[i];  vY[2] += py[i];
  }
  double solX[3], solY[3], mCopy[3][3];
  memcpy(mCopy, m, sizeof(mCopy));
  bool okX = solve3x3(mCopy, vX, solX);
  memcpy(mCopy, m, sizeof(mCopy));
  bool okY = solve3x3(mCopy, vY, solY);
  if (!okX || !okY) {
    Serial.println("!! calibration failed to solve - keeping previous values");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawCenterString("Calibration failed", W / 2, H / 2);
    delay(2000);
    return;
  }
  cal.ax = solX[0]; cal.bx = solX[1]; cal.cx = solX[2];
  cal.ay = solY[0]; cal.by = solY[1]; cal.cy = solY[2];
  float worst = 0;
  for (int i = 0; i < CAL_POINTS; i++) {
    float fx = cal.ax*rx[i] + cal.bx*ry[i] + cal.cx;
    float fy = cal.ay*rx[i] + cal.by*ry[i] + cal.cy;
    float e = sqrtf((fx-px[i])*(fx-px[i]) + (fy-py[i])*(fy-py[i]));
    if (e > worst) worst = e;
  }
  Serial.println("--- calibration result ---");
  Serial.printf("Calibration cal = { %.6ff, %.6ff, %.3ff,\n", cal.ax, cal.bx, cal.cx);
  Serial.printf("                    %.6ff, %.6ff, %.3ff };\n", cal.ay, cal.by, cal.cy);
  Serial.printf("worst residual: %.1f px%s\n", worst,
                worst > 12.0f ? "  (high - consider redoing)" : "  (good)");
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(worst > 12.0f ? TFT_YELLOW : TFT_GREEN, TFT_BLACK);
  char done[48];
  snprintf(done, sizeof(done), "Calibrated  (%.1f px error)", worst);
  tft.drawCenterString(done, W / 2, H / 2 - tft.fontHeight() / 2);
  delay(1200);
}

int currentTab = 0;     
static inline int tabCount(void) { return 1 + knownNodeCount; }
static String tabLabel(int i) {
  if (i == 0) return "Home";
  return "Xiao " + String(i);
}
#define NAV_H      56
#define COL_BG     0x0000
#define COL_PANEL  0x18E3
#define COL_ACTIVE 0x001F
#define COL_TEXT   0xFFFF
#define COL_DIM    0xAD55
#define COL_OK     0x07E0
#define COL_WARN   0xFD20
#define COL_STALE  0xF800
void drawNavBar(void) {
  const int n = tabCount();
  const int32_t tabW = tft.width() / n;
  
  const uint8_t textSize = (tabW >= 96) ? 2 : 1;
  for (int i = 0; i < n; i++) {
    const int32_t x = i * tabW;
    
    const int32_t w = (i == n - 1) ? (tft.width() - x) : tabW;
    const bool active = (currentTab == i);
    tft.fillRect(x, 0, w, NAV_H, active ? COL_ACTIVE : COL_PANEL);
    tft.drawRect(x, 0, w, NAV_H, COL_BG);
    tft.setTextSize(textSize);
    tft.setTextColor(active ? COL_TEXT : COL_DIM);
    String label = tabLabel(i);
    tft.drawCenterString(label, x + w / 2, NAV_H / 2 - tft.fontHeight() / 2);
    
    if (i > 0) {
      const NodeReading &nd = nodes[i - 1];
      tft.fillCircle(x + w / 2, NAV_H - 14, 3, nodeIsStale(nd) ? COL_STALE : COL_OK);
    }
    if (active) tft.fillRect(x, NAV_H - 4, w, 4, COL_OK);
  }
}
static void drawRow(int32_t y, const char *label, const String &value, uint16_t valueColor) {
  tft.setTextSize(2);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setCursor(20, y);
  tft.print(label);
  tft.setTextColor(valueColor, COL_BG);
  tft.setCursor(200, y);
  tft.print(value);
}
void drawHomeScreen(void) {
  tft.setTextSize(2);
  if (knownNodeCount == 0) {
    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawCenterString("Waiting for sensor nodes...", tft.width() / 2, NAV_H + 90);
    tft.drawCenterString("mesh: " MESH_SSID, tft.width() / 2, NAV_H + 120);
    return;
  }
  
  tft.setTextColor(COL_DIM, COL_BG);
  tft.setCursor(20, NAV_H + 14);  tft.print("NODE");
  tft.setCursor(150, NAV_H + 14); tft.print("TEMP");
  tft.setCursor(250, NAV_H + 14); tft.print("HUM");
  tft.setCursor(350, NAV_H + 14); tft.print("STATUS");
  for (int i = 0; i < knownNodeCount; i++) {
    const int32_t y = NAV_H + 48 + i * 32;
    const bool stale = nodeIsStale(nodes[i]);
    tft.setTextColor(COL_TEXT, COL_BG);
    tft.setCursor(20, y);
    tft.print(nodes[i].name);
    if (nodes[i].everSeen && nodes[i].dhtOk) {
      tft.setTextColor(stale ? COL_DIM : COL_TEXT, COL_BG);
      tft.setCursor(150, y); tft.printf("%.1fC", nodes[i].temp);
      tft.setCursor(250, y); tft.printf("%.0f%%", nodes[i].hum);
    } else if (nodes[i].everSeen) {
      
      
      tft.setTextColor(COL_WARN, COL_BG);
      tft.setCursor(150, y); tft.print("DHT?");
      tft.setCursor(250, y); tft.print("--");
    } else {
      tft.setTextColor(COL_DIM, COL_BG);
      tft.setCursor(150, y); tft.print("--");
      tft.setCursor(250, y); tft.print("--");
    }
    tft.setTextColor(stale ? COL_STALE : COL_OK, COL_BG);
    tft.setCursor(350, y);
    tft.print(stale ? "STALE" : "LIVE");
  }
}
void drawNodeDetailScreen(int idx) {
  if (idx < 0 || idx >= knownNodeCount) return;
  tft.setTextSize(3);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setCursor(20, NAV_H + 16);
  tft.print(nodes[idx].name);
  if (!nodes[idx].everSeen) {
    tft.setTextSize(2);
    tft.setTextColor(COL_DIM, COL_BG);
    tft.drawCenterString("No data from this node yet", tft.width() / 2, NAV_H + 110);
    return;
  }
  const NodeReading &n = nodes[idx];
  const bool stale = nodeIsStale(n);
  const uint16_t vc = stale ? COL_DIM : COL_TEXT;
  
  
  
  const unsigned long age = (millis() - n.lastSeenMs) / 1000;
  char status[48];
  snprintf(status, sizeof(status), "%s  (%lus ago)",
           stale ? "STALE" : "live", age);
  tft.setTextSize(2);
  tft.setTextColor(stale ? COL_STALE : COL_OK, COL_BG);
  tft.setCursor(20, NAV_H + 48);
  tft.print(status);
  drawRow(NAV_H + 90,  "Temp",
          n.dhtOk ? String(n.temp, 1) + " C" : String("sensor fault"),
          n.dhtOk ? vc : COL_WARN);
  drawRow(NAV_H + 120, "Humidity",
          n.dhtOk ? String(n.hum, 1) + " %"  : String("sensor fault"),
          n.dhtOk ? vc : COL_WARN);
  drawRow(NAV_H + 150, "IR / heat", String(n.ir),  vc);
  drawRow(NAV_H + 180, "Gas",       String(n.gas), vc);
  drawRow(NAV_H + 210, "Soil",      String(n.soil), vc);
  
}
void drawStatusBar(void) {
  const int32_t y = tft.height() - 30;
  tft.fillRect(0, y - 4, tft.width(), 34, COL_BG);
  int live = 0;
  for (int i = 0; i < knownNodeCount; i++) if (!nodeIsStale(nodes[i])) live++;
  char buf[64];
  snprintf(buf, sizeof(buf), "mesh %u nodes  |  %d/%d live  |  up %lus",
           (unsigned)mesh.getNodeList().size(), live, knownNodeCount,
           millis() / 1000);
  tft.setTextSize(2);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawCenterString(buf, tft.width() / 2, y);
}
void drawCurrentScreen(void) {
  
  
  if (currentTab >= tabCount()) currentTab = 0;
  tft.fillRect(0, NAV_H, tft.width(), tft.height() - NAV_H, COL_BG);
  drawNavBar();
  if (currentTab == 0) drawHomeScreen();
  else                 drawNodeDetailScreen(currentTab - 1);
  drawStatusBar();
}
bool handleNavTap(int32_t x, int32_t y) {
  if (y >= NAV_H) return false;
  const int n = tabCount();
  const int32_t tabW = tft.width() / n;
  int idx = constrain((int)(x / tabW), 0, n - 1);
  if (idx != currentTab) {
    currentTab = idx;
    drawCurrentScreen();
    Serial.printf("screen -> %s\n", tabLabel(idx).c_str());
  }
  return true;
}

void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("rx from %u: %s\n", from, msg.c_str());
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, msg)) return;
  String name = doc["node"] | "unknown";
  int idx = findOrAddNode(name);
  if (idx < 0) return;   
  nodes[idx].ir   = doc["ir"]   | 0;
  nodes[idx].gas  = doc["gas"]  | 0;
  nodes[idx].soil = doc["soil"] | 0;
  nodes[idx].temp = doc["temp"] | 0.0f;
  nodes[idx].hum  = doc["hum"]  | 0.0f;
  
  nodes[idx].dhtOk = doc["dhtok"] | true;
  nodes[idx].lastSeenMs = millis();
  nodes[idx].everSeen   = true;
  nodesDirty = true;
}
void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("mesh: new connection %u\n", nodeId);
  nodesDirty = true;
}
void changedConnectionCallback(void) {
  Serial.println("mesh: connections changed");
  nodesDirty = true;
}

void setup(void) {
  Serial.begin(115200);
  delay(500);
  Serial.println("--- forest display host booting ---");
  
  
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  pinMode(TOUCH_IRQ, INPUT_PULLUP);
  SPI.begin(D4, TOUCH_MISO, D10, TOUCH_CS);
  tft.init();
  tft.setRotation(1);            
  tft.fillScreen(COL_BG);
#if !SKIP_CALIBRATION
  calibrate();
#endif
  tft.fillScreen(COL_BG);
  tft.setTextSize(2);
  tft.setTextColor(COL_DIM, COL_BG);
  tft.drawCenterString("Joining mesh...", tft.width() / 2, tft.height() / 2);
  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  drawCurrentScreen();
  Serial.println("ready - joined mesh, tap the tabs to switch screens");
}
void loop(void) {
  mesh.update();     
  int32_t x, y;
  if (getTap(&x, &y)) {
    Serial.printf("tap: %ld, %ld\n", (long)x, (long)y);
    handleNavTap(x, y);
  }
  
  
  static unsigned long lastPaint = 0;
  if (nodesDirty || millis() - lastPaint > 1000) {
    nodesDirty = false;
    lastPaint  = millis();
    drawCurrentScreen();
  }
}