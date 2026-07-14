#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── Pin definitions ──────────────────────────────────────────────
#define THERMISTOR_PIN  D0
#define BUZZER_PIN      D6

// ── OLED setup ───────────────────────────────────────────────────
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C   // most 128x32 SSD1306 boards use 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ── Two-point calibration model ──────────────────────────────────
// Point 1: ADC =  56  →  30°C
// Point 2: ADC = 157  →  80°C
// Slope = (80 - 30) / (157 - 56) = 0.4950°C per ADC unit
// Formula: Temperature = 0.4950 × Value + 2.28
// Example readings:
//   30°C → ADC ≈  56  |  50°C → ADC ≈  96
//   60°C → ADC ≈ 117  |  70°C → ADC ≈ 137
float rawToTemp(int raw) {
  return 0.4950 * raw + 2.28;
}

// ── State variables ──────────────────────────────────────────────
int   rawValue    = 0;
float temperature = 0.0;
bool  showTemp    = true;   // toggles between temp and raw display

unsigned long lastRead   = 0;   // tracks last sensor read  (every 2 s)
unsigned long lastToggle = 0;   // tracks last display swap  (every 1 s)

// ── Setup ────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Start I2C on D4 (SDA) and D5 (SCL)
  Wire.begin(D4, D5);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED not found – check wiring");
    while (true);   // halt
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();

  Serial.println("Thermistor + OLED ready");
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // Read thermistor every 2 seconds
  if (now - lastRead >= 2000) {
    lastRead  = now;
    rawValue  = analogRead(THERMISTOR_PIN);
    temperature = rawToTemp(rawValue);

    Serial.print("Raw: ");
    Serial.print(rawValue);
    Serial.print("  |  Temp: ");
    Serial.print(temperature, 1);
    Serial.println(" C");
  }

  // Alternate display every 1 second
  if (now - lastToggle >= 1000) {
    lastToggle = now;
    showTemp   = !showTemp;

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);

    if (showTemp) {
      display.println("Temperature:");
      display.setTextSize(2);
      display.setCursor(0, 12);
      display.print(temperature, 1);
      display.print(" C");
    } else {
      display.println("Raw value:");
      display.setTextSize(2);
      display.setCursor(0, 12);
      display.print(rawValue);
    }

    display.display();
  }

  // Buzzer: on if temperature exceeds 50 C
  if (temperature > 50.0) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}
