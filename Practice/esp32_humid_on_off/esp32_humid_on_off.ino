// ============ ESP32-S3 sketch matching Nano external-pullup logic ============
#include <Arduino.h>
#include <EEPROM.h>

/* ======= PINS (ESP32-S3) ======= */
// Floats (EXTERNAL pull-ups to 3.3V; switch to GND)
const int FLOAT_TOP_PIN    = 7;     // TOP float (LOW when closed to GND)
const int FLOAT_BOTTOM_PIN = 16;    // BOTTOM float (LOW when closed to GND)

// Relays
const int RELAY_PUMP_PIN   = 17;    // pump relay IN
const int RELAY_HUMID_PIN  = 15;    // humidifier relay IN

// Per-relay polarity (same semantics as Nano)
const bool RELAY_PUMP_ACTIVE_LOW  = false;   // false=active-HIGH, true=active-LOW
const bool RELAY_HUMID_ACTIVE_LOW = true;    // humidifier module active-LOW

// Shunt sense (Node X, pump side of 1Ω) on ADC1
const int  SHUNT_ADC_PIN   = 4;     // ESP32 ADC1 channel

// Optional CLEAR button (EXTERNAL pull-up to 3.3V; button to GND)
const bool ENABLE_CLEAR_BTN = false;
const int  CLEAR_BTN_PIN    = 8;

/* ======= NOISE / TIMING GUARDS ======= */
const uint16_t FLOAT_FILTER_MS   = 150;      // must be stable this long
const uint32_t MIN_DWELL_MS      = 3000;     // min ON/OFF runtime (anti-chatter)
const uint32_t FAULT_BLANK_MS    = 700;      // ignore short-detect after pump ON

/* ======= SHORT-DETECT ======= */
const uint16_t SHORT_mV    = 400;            // >400 mV across 1Ω => fault
const uint16_t SHORT_MS    = 80;             // must exceed threshold for this long
const uint16_t ADC_SAMPLES = 12;             // averaging samples

/* ======= PERSISTENT FAULT LATCH (EEPROM) ======= */
const int EE_ADDR_FAULT      = 0;            // 1 byte
const uint8_t EE_FAULT_CLEAR = 0x00;
const uint8_t EE_FAULT_SET   = 0xF1;
const size_t EEPROM_BYTES    = 8;

/* ======= STATE ======= */
bool pumpRunning  = false;
bool faultLatched = false;  // loaded from EEPROM
uint32_t lastSwitchMs = 0;
uint32_t pumpOnAtMs   = 0;

/* ======= HELPERS ======= */
inline void setRelayLevel(int pin, bool on, bool activeLow) {
  digitalWrite(pin, activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}
inline void relayPumpOn()        { setRelayLevel(RELAY_PUMP_PIN,  true,  RELAY_PUMP_ACTIVE_LOW); }
inline void relayPumpOff()       { setRelayLevel(RELAY_PUMP_PIN,  false, RELAY_PUMP_ACTIVE_LOW); }
inline void setHumid(bool on)    { setRelayLevel(RELAY_HUMID_PIN, on,    RELAY_HUMID_ACTIVE_LOW); }

struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p){
    pin = p;
    pinMode(pin, INPUT);                    // <<< NO INTERNAL PULLUP >>>
    stable = candidate = digitalRead(pin);
    tChange = millis();
  }
  int read(uint16_t hold_ms){
    int raw = digitalRead(pin);
    if (raw != candidate) { candidate = raw; tChange = millis(); }
    if ((millis() - tChange) >= hold_ms && stable != candidate) stable = candidate;
    return stable;
  }
};
Debounce topF, botF;

static void eepromInit(){ EEPROM.begin(EEPROM_BYTES); }
static void saveFaultToEEPROM(bool set){
  uint8_t want = set ? EE_FAULT_SET : EE_FAULT_CLEAR;
  if (EEPROM.read(EE_ADDR_FAULT) != want) { EEPROM.write(EE_ADDR_FAULT, want); EEPROM.commit(); }
}
static bool faultStoredInEEPROM(){ return EEPROM.read(EE_ADDR_FAULT) == EE_FAULT_SET; }

/* ======= NANO-STYLE SHUNT READING (changed) ======= */
// Treat ADC full-scale as ~1.1 V (like Nano INTERNAL ref)
static const float VREF_mV = 1100.0f;   // tweak to calibrate if needed
// Optional: uncomment to force 12-bit width globally
// void analogReadResolution(int bits);  // forward
// (Arduino-ESP32 supports analogReadResolution)
uint32_t readShunt_mV() {
  uint32_t acc = 0;
  for (uint16_t i = 0; i < ADC_SAMPLES; ++i) {
    int raw = analogRead(SHUNT_ADC_PIN);        // 0..4095 (12-bit)
    acc += (uint32_t)raw;
    delayMicroseconds(200);
  }
  float avg = acc / (float)ADC_SAMPLES;         // average raw counts
  // Scale to millivolts using Nano-like 1.1 V FS and 12-bit span
  float mv = (avg * VREF_mV) / 4095.0f;
  return (uint32_t)(mv + 0.5f);
}

void applyPumpState(bool on) {
  if (on == pumpRunning) return;
  if (millis() - lastSwitchMs < MIN_DWELL_MS) return;  // dwell guard
  if (on) {
    relayPumpOn();
    pumpOnAtMs = millis();
    Serial.println(F("PUMP -> ON (latched)"));
  } else {
    relayPumpOff();
    Serial.println(F("PUMP -> OFF (latched)"));
  }
  pumpRunning = on;
  lastSwitchMs = millis();
}

bool checkClearButtonOnce(){
  if (!ENABLE_CLEAR_BTN) return false;
  static int last=HIGH; static uint32_t t=0;
  int v = digitalRead(CLEAR_BTN_PIN);
  if (v != last){ last=v; t=millis(); }
  if (last==LOW && millis()-t>20){ while(digitalRead(CLEAR_BTN_PIN)==LOW) delay(5); return true; }
  return false;
}

/* ======= SETUP ======= */
void setup() {
  Serial.begin(115200);
  delay(800); // let USB-CDC enumerate

  // Inputs with EXTERNAL pull-ups (to 3.3V)
  topF.begin(FLOAT_TOP_PIN);
  botF.begin(FLOAT_BOTTOM_PIN);
  if (ENABLE_CLEAR_BTN) pinMode(CLEAR_BTN_PIN, INPUT); // no internal pull-up

  // Outputs
  pinMode(RELAY_PUMP_PIN,  OUTPUT);
  pinMode(RELAY_HUMID_PIN, OUTPUT);
  relayPumpOff();
  setHumid(true);     // ON unless fault

  // ADC config — mimic Nano 1.1V span
  pinMode(SHUNT_ADC_PIN, INPUT_PULLDOWN);       // keep Node X from floating when pump OFF
  analogSetPinAttenuation(SHUNT_ADC_PIN, ADC_0db); // ~1.1V FS
  // Optional: lock width to 12-bit so 0..4095 math stays true
  analogReadResolution(12);

  // EEPROM
  eepromInit();
  faultLatched = faultStoredInEEPROM();

  Serial.println(F("\nESP32-S3: Nano-equivalent logic (external pull-ups)"));
  Serial.print(F("Pump relay trigger: "));  Serial.println(RELAY_PUMP_ACTIVE_LOW  ? F("LOW") : F("HIGH"));
  Serial.print(F("Humid relay trigger: ")); Serial.println(RELAY_HUMID_ACTIVE_LOW ? F("LOW") : F("HIGH"));
  if (faultLatched) Serial.println(F("Persistent FAULT latched: pump inhibited until CLEAR."));
  Serial.println(F("To CLEAR fault: send 'C' or press CLEAR button (if enabled)."));
}

/* ======= LOOP ======= */
void loop() {
  // Manual clear
  if (ENABLE_CLEAR_BTN && checkClearButtonOnce()){
    faultLatched = false; saveFaultToEEPROM(false);
    Serial.println(F("Fault CLEARED via button."));
  }
  if (Serial.available()){
    char c = Serial.read();
    if (c=='C'){ faultLatched=false; saveFaultToEEPROM(false); Serial.println(F("Fault CLEARED via Serial.")); }
  }

  // Debounced floats (LOW = closed to GND; HIGH = open via external pull-up)
  int top    = topF.read(FLOAT_FILTER_MS);      // TOP: LOW = UP (FULL)
  int bottom = botF.read(FLOAT_FILTER_MS);      // BOTTOM: HIGH = DOWN (LOW level)

  if (faultLatched) {
    if (pumpRunning) applyPumpState(false);
    setHumid(false); // OFF on fault
  } else {
    // === CONTROL WITHOUT ONE-SHOT (mirror Nano) ===
    if (!pumpRunning) {
      if (top == LOW) { // your Nano "start on top LOW"
        applyPumpState(true);
        Serial.println(F("Reason: TOP UP -> start pump"));
      }
    } else {
      if (bottom == HIGH) { // bottom open -> stop
        applyPumpState(false);
        Serial.println(F("Reason: BOTTOM DOWN -> stop pump"));
      }
    }

    // Short-detect with inrush blanking (now using Nano-like scaling)
    static uint32_t overStart = 0;
    if (pumpRunning) {
      if (millis() - pumpOnAtMs >= FAULT_BLANK_MS) {
        uint32_t mv = readShunt_mV();                 // <-- changed function
        if (mv > SHORT_mV) {
          if (overStart == 0) overStart = millis();
          if (millis() - overStart >= SHORT_MS) {
            Serial.print(F("FAULT: Vshunt=")); Serial.print(mv); Serial.println(F(" mV > 400 mV"));
            applyPumpState(false);
            faultLatched = true;
            saveFaultToEEPROM(true);
            Serial.println(F("Fault LATCHED and STORED. Pump inhibited until CLEAR."));
          }
        } else {
          overStart = 0;
        }
      } else {
        overStart = 0; // ignore inrush
      }
    }

    setHumid(!faultLatched); // ON unless fault
  }

  // Debug (1 Hz)
  static uint32_t lastDbg=0;
  if (millis() - lastDbg > 1000) {
    lastDbg = millis();
    Serial.print(F("Pump="));  Serial.print(pumpRunning?F("ON"):F("OFF"));
    Serial.print(F("  FaultLatched=")); Serial.print(faultLatched?F("YES"):F("NO"));
    uint32_t mv = pumpRunning ? readShunt_mV() : 0;
    Serial.print(F("  Vshunt=")); Serial.print(mv); Serial.println(F(" mV"));
  }

  delay(5);
}
