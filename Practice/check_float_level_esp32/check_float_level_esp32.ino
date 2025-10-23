// ESP32-S3 Float Sensor Checker
// - Default: EXTERNAL pull-ups to 3.3V; float switch closes to GND (active LOW)
// - Prints state every 500 ms + edge notifications with debounce

#include <Arduino.h>

// ====== CONFIG ======
const int FLOAT_TOP_PIN    = 7;   // GPIO7  (TOP float)
const int FLOAT_BOTTOM_PIN = 16;  // GPIO16 (BOTTOM float)

// If you don't have external pull-ups to 3.3V, set this to true:
const bool USE_INTERNAL_PULLUPS = false;

// How long an input must be stable to count a change (debounce)
const uint16_t DEBOUNCE_MS = 120;

// ====== Debounce helper ======
struct DebounceIn {
  int pin;
  int stable, cand;
  uint32_t t;
  void begin(int p, bool usePullup){
    pin = p;
    pinMode(pin, usePullup ? INPUT_PULLUP : INPUT);
    stable = cand = digitalRead(pin);
    t = millis();
  }
  // return debounced state (HIGH/LOW)
  int read(){
    int raw = digitalRead(pin);
    if (raw != cand) { cand = raw; t = millis(); }
    if ((millis() - t) >= DEBOUNCE_MS && stable != cand) stable = cand;
    return stable;
  }
};

DebounceIn topF, botF;

// Map logic to human words
const char* upDown(int v, bool isTop){
  // Our convention: switch CLOSED to GND = LOW
  // For both floats, LOW = "UP" (engaged), HIGH = "DOWN" (released)
  // If your hardware semantics differ, change labels here.
  if (v == LOW)  return "UP   (LOW/closed)";
  else           return "DOWN (HIGH/open)";
}

void setup() {
  Serial.begin(115200);
  delay(800);  // allow USB-CDC to enumerate

  topF.begin(FLOAT_TOP_PIN,    USE_INTERNAL_PULLUPS);
  botF.begin(FLOAT_BOTTOM_PIN, USE_INTERNAL_PULLUPS);

  Serial.println("\nESP32-S3 Float Sensor Checker");
  Serial.print("Pins: TOP="); Serial.print(FLOAT_TOP_PIN);
  Serial.print("  BOTTOM="); Serial.println(FLOAT_BOTTOM_PIN);
  Serial.print("Pull-ups: "); Serial.println(USE_INTERNAL_PULLUPS ? "Internal" : "External (to 3.3V)");
  Serial.println("Expected wiring: float closes to GND => reads LOW.");
  Serial.println("----");
}

void loop() {
  static int lastTop = -1, lastBot = -1;
  static uint32_t lastPrint = 0;

  int top = topF.read();
  int bot = botF.read();

  // Edge notifications (only when they change)
  if (top != lastTop) {
    lastTop = top;
    Serial.print("[EDGE] TOP -> "); Serial.println(upDown(top, true));
  }
  if (bot != lastBot) {
    lastBot = bot;
    Serial.print("[EDGE] BOTTOM -> "); Serial.println(upDown(bot, false));
  }

  // Periodic status line
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    Serial.print("TOP=");    Serial.print(upDown(top, true));
    Serial.print(" | BOTTOM="); Serial.println(upDown(bot, false));
  }

  delay(10);
}
