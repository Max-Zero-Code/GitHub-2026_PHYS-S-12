// ── ESP32 Dev V1 — MESSAGE SENDER ────────────────────────────────
// Type a message in Serial Monitor → sent to Xiao via ESP-NOW
// Xiao displays it on its 128x32 OLED screen.
// ─────────────────────────────────────────────────────────────────

#include <esp_now.h>
#include <WiFi.h>

// ── Xiao MAC address: 94:A9:90:6D:48:44 ─────────────────────────
uint8_t xiaoAddress[] = {0x94, 0xA9, 0x90, 0x6D, 0x48, 0x44};

// ── Message structure ─────────────────────────────────────────────
typedef struct Message {
  char text[64];  // up to 63 characters + null terminator
} Message;

Message outgoing;

// ── Send callback ─────────────────────────────────────────────────
void OnDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivered." : "Delivery failed.");
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed — halting");
    while (true);
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, xiaoAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add Xiao as peer — halting");
    while (true);
  }

  Serial.println("Ready. Type a message and press Enter:");
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) return;

    // Copy into struct (max 63 chars)
    memset(outgoing.text, 0, sizeof(outgoing.text));
    strncpy(outgoing.text, input.c_str(), 63);

    Serial.print("Sending: \"");
    Serial.print(outgoing.text);
    Serial.println("\"");

    esp_now_send(xiaoAddress, (uint8_t *)&outgoing, sizeof(outgoing));
  }
}
