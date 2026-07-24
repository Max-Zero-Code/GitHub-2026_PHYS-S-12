// ── Xiao ESP32C3 — MESSAGE DISPLAY ───────────────────────────────
// Receives a message from ESP32 Dev V1 via ESP-NOW and displays
// it on the 128x32 SSD1306 OLED screen.
// ─────────────────────────────────────────────────────────────────

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED setup ────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  32
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Message structure (must match sender exactly) ─────────────────
typedef struct Message {
  char text[64];
} Message;

Message incoming;
bool newMessage = false;

// ── Receive callback (core 3.x signature) ────────────────────────
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  memcpy(&incoming, data, sizeof(incoming));
  incoming.text[63] = '\0';  // ensure null-terminated
  newMessage = true;
  Serial.print("Received: ");
  Serial.println(incoming.text);
}

// ── Show message on OLED with word wrap ───────────────────────────
void showOnOLED(const char *msg) {
  display.clearDisplay();
  display.setTextSize(1);       // 6x8px per char → 21 chars wide, 4 lines tall
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextWrap(true);    // auto-wrap at screen edge
  display.print(msg);
  display.display();
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // I2C on D4 (SDA) and D5 (SCL)
  Wire.begin(D4, D5);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED not found — check wiring");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Waiting for");
  display.println("message...");
  display.display();

  WiFi.mode(WIFI_STA);
  Serial.print("Xiao MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed — halting");
    while (true);
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Listening...");
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  if (newMessage) {
    newMessage = false;
    showOnOLED(incoming.text);
  }
}
