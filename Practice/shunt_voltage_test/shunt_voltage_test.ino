/*
  Read 0.3–0.5 V on Arduino Nano/UNO A0
  - Uses INTERNAL 1.1 V reference for better resolution
  - Prints voltage in mV and optional current through a shunt
*/

const uint8_t SHUNT_PIN   = A0;       // your input
const float   VREF_mV     = 1100.0;   // internal ref (tweak after calibrating)
const float   SHUNT_OHMS  = 1.0;      // set to your shunt value; 0.0 to hide current
const uint8_t ADC_SAMPLES = 16;       // averaging for noise
const uint32_t PRINT_MS   = 250;      // update rate

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  analogReference(INTERNAL);      // ~1.1 V full-scale
  delay(10);
  (void)analogRead(SHUNT_PIN);    // throw away first read

  Serial.println(F("A0 reader (INTERNAL 1.1V)"));
  Serial.print(F("Resolution ~ ")); 
  Serial.print(VREF_mV/1023.0, 3);
  Serial.println(F(" mV/LSB"));
}

uint16_t readADCavg(uint8_t pin, uint8_t n) {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < n; i++) {
    acc += analogRead(pin);
    delayMicroseconds(200);
  }
  return (uint16_t)(acc / n);
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= PRINT_MS) {
    last = millis();

    uint16_t adc = readADCavg(SHUNT_PIN, ADC_SAMPLES);
    float mV = adc * (VREF_mV / 1023.0f);

    Serial.print(F("ADC=")); Serial.print(adc);
    Serial.print(F("  V=")); Serial.print(mV, 2); Serial.print(F(" mV"));

    if (SHUNT_OHMS > 0.0f) {
      float mA = mV / SHUNT_OHMS;    // I(mA) = V(mV)/R(ohm)
      Serial.print(F("  I=")); Serial.print(mA, 2); Serial.print(F(" mA"));
    }
    Serial.println();
  }
}
