#include <Arduino.h>

// Float switches
const int FLOAT_TOP_PIN    = 7;   // GPIO7  (open = HIGH, closed-to-GND = LOW)
const int FLOAT_BOTTOM_PIN = 16;  // GPIO16 (open = HIGH, closed-to-GND = LOW)

// Relay input
const int  RELAY_PIN         = 17;    // ESP32-S3 GPIO -> relay IN1
const bool RELAY_ACTIVE_LOW  = false; // set true if your board is LOW-trigger (LOW=ON)

inline void relayOn()  { digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW  : HIGH); }
inline void relayOff() { digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW ); }

// (optional) tiny debounce helper
int readStable(int pin) {
  int a = digitalRead(pin);
  delay(5);
  int b = digitalRead(pin);
  return (a == b) ? a : digitalRead(pin);
}

// Latch state: are we currently running the pump?
bool pumpRunning = false;

void applyPumpState(bool on) {
  if (on) relayOn(); else relayOff();
  if (on != pumpRunning) {
    pumpRunning = on;
    Serial.println(on ? "PUMP -> ON (latched)" : "PUMP -> OFF (latched)");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(FLOAT_TOP_PIN,    INPUT_PULLUP);  // open = HIGH, closed-to-GND = LOW
  pinMode(FLOAT_BOTTOM_PIN, INPUT_PULLUP);

  pinMode(RELAY_PIN, OUTPUT);
  applyPumpState(false); // safe default (OFF)

  Serial.println("\nTwo-float with HYSTERESIS (latch):");
  Serial.println(" - Start when TOP is UP   (TOP == LOW)");
  Serial.println(" - Keep ON until BOTTOM is DOWN (BOTTOM == HIGH)");
  Serial.println(" - Then stop and wait for TOP to be UP again");
}

void loop() {
  // Read (debounced)
  int topLevel    = readStable(FLOAT_TOP_PIN);    // HIGH = open (down), LOW = closed (up)
  int bottomLevel = readStable(FLOAT_BOTTOM_PIN); // HIGH = open (down), LOW = closed (up)

  // --- LATCH LOGIC ---
  if (!pumpRunning) {
    // Currently OFF: only one condition can start us — TOP float UP (closed -> LOW)
    if (topLevel == LOW) {
      applyPumpState(true);  // Start pumping
      Serial.println("Reason: TOP UP (LOW) -> starting pump.");
    }
  } else {
    // Currently ON: keep running until BOTTOM goes DOWN (open -> HIGH)
    if (bottomLevel == HIGH) {
      applyPumpState(false); // Stop pumping
      Serial.println("Reason: BOTTOM DOWN (HIGH) -> stopping pump.");
    }
  }
  delay(50);
}
