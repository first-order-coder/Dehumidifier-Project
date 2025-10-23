#include <Arduino.h>

// Float switches
const int FLOAT_TOP_PIN    = 7;   // GPIO7
const int FLOAT_BOTTOM_PIN = 16;  // GPIO16

// Relay input
const int RELAY_PIN         = 17;    // ESP32-S3 GPIO -> relay IN1
const bool RELAY_ACTIVE_LOW = false; // set true if your board is LOW-trigger (LOW=ON)

inline void relayOn()  { digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW  : HIGH); }
inline void relayOff() { digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW ); }

// (optional) tiny debounce helper
int readStable(int pin) {
  int a = digitalRead(pin);
  delay(5);
  int b = digitalRead(pin);
  return (a == b) ? a : digitalRead(pin);
}

void setup() {
  Serial.begin(115200);

  pinMode(FLOAT_TOP_PIN,    INPUT_PULLUP);  // open = HIGH, closed-to-GND = LOW
  pinMode(FLOAT_BOTTOM_PIN, INPUT_PULLUP);

  pinMode(RELAY_PIN, OUTPUT);
  relayOff(); // safe default

  Serial.println("\nTwo-float + relay:");
  Serial.println(" - both LOW  = FULL  -> Pump ON");
  Serial.println(" - both HIGH = EMPTY -> Pump OFF");
  Serial.println(" - mixed     = MID   -> Pump OFF");
}

void loop() {
  // read (with tiny debounce)
  int topLevel    = readStable(FLOAT_TOP_PIN);    // HIGH=open (float down), LOW=closed (float up)
  int bottomLevel = readStable(FLOAT_BOTTOM_PIN);

  bool bothHigh = (topLevel == HIGH) && (bottomLevel == HIGH); // both floats down -> EMPTY
  bool bothLow  = (topLevel == LOW)  && (bottomLevel == LOW);  // both floats up   -> FULL

  if (bothLow) {
    relayOn();
    Serial.println("TANK FULL (both LOW)  -> Pump ON (relay energized)");
  } else if (bothHigh) {
    relayOff();
    Serial.println("TANK EMPTY (both HIGH) -> Pump OFF");
  } else {
    relayOn();
    Serial.println("TANK MID-LEVEL (one HIGH, one LOW) -> Pump ON");
  }

  delay(250);
}
