#include <Arduino.h>

const int FLOAT_TOP_PIN    = 7;  // top float switch -> GPIO7
const int FLOAT_BOTTOM_PIN = 16;  // bottom float switch -> GPIO9

void setup() {
  Serial.begin(115200);
  pinMode(FLOAT_TOP_PIN,    INPUT_PULLUP);  // open = HIGH, closed-to-GND = LOW
  pinMode(FLOAT_BOTTOM_PIN, INPUT_PULLUP);
  Serial.println("\nTwo-float level check (both HIGH = EMPTY, both LOW = FULL)");
}

void loop() {
  int topLevel    = digitalRead(FLOAT_TOP_PIN);    // HIGH=open (down), LOW=closed (up)
  int bottomLevel = digitalRead(FLOAT_BOTTOM_PIN);

  if (topLevel == HIGH && bottomLevel == HIGH) {
    Serial.println("TANK EMPTY (both floats DOWN -> both HIGH)");
  } else if (topLevel == LOW && bottomLevel == LOW) {
    Serial.println("TANK FULL (both floats UP -> both LOW)");
  } else {
    // One up, one down = intermediate level
    Serial.println("TANK MID-LEVEL (one HIGH, one LOW)");
  }

  delay(250);
}
