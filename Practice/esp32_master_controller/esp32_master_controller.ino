#include <Arduino.h>
#include <EEPROM.h>

/* ======= PINS (Nano/UNO) ======= */
const int FLOAT_TOP_PIN    = 2;   // open = HIGH, closed-to-GND = LOW
const int FLOAT_BOTTOM_PIN = 3;   // open = HIGH, closed-to-GND = LOW

const int RELAY_PUMP_PIN   = 8;   // pump relay IN/driver
const int RELAY_HUMID_PIN  = 9;   // humidifier relay IN/driver

// >>> Polarity per relay (changed here)
const bool RELAY_PUMP_ACTIVE_LOW  = false;  // pump module: false=active-HIGH, true=active-LOW
const bool RELAY_HUMID_ACTIVE_LOW = true;   // humidifier module: REVERSED (active-LOW)

/* ======= ADC ======= */
const int  SHUNT_ADC_PIN    = A0;  // Node X (pump side of 1Ω)
const bool USE_INTERNAL_ADC_REF = true; // we use the internal ADC ref for better resolution on small voltages

/* ======= NOISE / TIMING GUARDS ======= */
const uint16_t FLOAT_FILTER_MS   = 150;     // must be stable this long
const uint32_t MIN_DWELL_MS      = 3000;    // min ON/OFF runtime (anti-chatter)
const uint32_t FAULT_BLANK_MS    = 700;     // ignore short-detect after pump ON

/* ======= SHORT-DETECT ======= */
const uint16_t SHORT_mV    = 400;           // >400 mV across 1Ω => fault
const uint16_t SHORT_MS    = 80;            // must exceed threshold for this long
const uint8_t  ADC_SAMPLES = 12;

/* ======= PERSISTENT FAULT LATCH (EEPROM) ======= */
const int EE_ADDR_FAULT = 0;            // 1 byte
const uint8_t EE_FAULT_CLEAR = 0x00;
const uint8_t EE_FAULT_SET   = 0xF1;

/* ======= OPTIONAL MANUAL CLEAR ======= */
const bool ENABLE_CLEAR_BTN = false;    // set true to use a button
const int  CLEAR_BTN_PIN    = 4;        // momentary button to GND (INPUT_PULLUP)

/* ======= STATE ======= */
bool pumpRunning  = false;
bool faultLatched = false;  // will be loaded from EEPROM on boot
bool armed        = true;   // one-shot start condition
uint32_t lastSwitchMs = 0;
uint32_t pumpOnAtMs   = 0;

/* ======= HELPERS ======= */
inline void setRelayLevel(int pin, bool on, bool activeLow) {
  digitalWrite(pin, (activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW)));
}
inline void relayPumpOn()        { setRelayLevel(RELAY_PUMP_PIN,  true,  RELAY_PUMP_ACTIVE_LOW); }
inline void relayPumpOff()       { setRelayLevel(RELAY_PUMP_PIN,  false, RELAY_PUMP_ACTIVE_LOW); }
inline void setHumid(bool on)    { setRelayLevel(RELAY_HUMID_PIN, on,    RELAY_HUMID_ACTIVE_LOW); }

struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p){ pin=p; pinMode(pin, INPUT_PULLUP); stable=candidate=digitalRead(pin); tChange=millis(); }
  int read(uint16_t hold_ms){
    int raw = digitalRead(pin);
    if (raw != candidate) { candidate = raw; tChange = millis(); }
    if ((millis() - tChange) >= hold_ms && stable != candidate) stable = candidate;
    return stable;
  }
};
Debounce topF, botF;

uint32_t readShunt_mV() {
  static bool refInit=false;
  if (!refInit) {
    analogReference(USE_INTERNAL_ADC_REF ? INTERNAL : DEFAULT);
    delay(5); (void)analogRead(SHUNT_ADC_PIN);
    refInit = true;
  }
  uint32_t acc=0;
  for (uint8_t i=0;i<ADC_SAMPLES;i++){ acc += analogRead(SHUNT_ADC_PIN); delayMicroseconds(200); }
  const float Vref_mV = USE_INTERNAL_ADC_REF ? 1100.0f : 5000.0f; // nominal
  return (uint32_t)((acc/(float)ADC_SAMPLES) * Vref_mV / 1023.0f + 0.5f);
}

void saveFaultToEEPROM(bool set){
  uint8_t want = set ? EE_FAULT_SET : EE_FAULT_CLEAR;
  if (EEPROM.read(EE_ADDR_FAULT) != want) EEPROM.update(EE_ADDR_FAULT, want);
}
bool faultStoredInEEPROM(){ return EEPROM.read(EE_ADDR_FAULT) == EE_FAULT_SET; }

void applyPumpState(bool on) {
  if (on == pumpRunning) return;
  if (millis() - lastSwitchMs < MIN_DWELL_MS) return; // dwell guard
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

  topF.begin(FLOAT_TOP_PIN);
  botF.begin(FLOAT_BOTTOM_PIN);

  pinMode(RELAY_PUMP_PIN,  OUTPUT);
  pinMode(RELAY_HUMID_PIN, OUTPUT);
  if (ENABLE_CLEAR_BTN) pinMode(CLEAR_BTN_PIN, INPUT_PULLUP);

  relayPumpOff();
  setHumid(true);     // (same behavior as before) ON unless fault

  // Load persistent fault
  faultLatched = faultStoredInEEPROM();
  if (faultLatched) {
    armed = false; // do not allow start
    Serial.println(F("\nPersistent FAULT latched: pump will NOT start until cleared."));
  } else {
    armed = true;
    Serial.println(F("\nNo fault stored: system ARMED (one-shot)."));
  }
  Serial.print(F("Pump relay trigger: "));  Serial.println(RELAY_PUMP_ACTIVE_LOW  ? F("LOW") : F("HIGH"));
  Serial.print(F("Humid relay trigger: ")); Serial.println(RELAY_HUMID_ACTIVE_LOW ? F("LOW") : F("HIGH"));
  Serial.println(F("To CLEAR fault: send 'C' over Serial or press the CLEAR button (if enabled)."));
}

/* ======= LOOP ======= */
void loop() {
  // Manual clear: button or Serial 'C'
  if (ENABLE_CLEAR_BTN && checkClearButtonOnce()){
    faultLatched = false; saveFaultToEEPROM(false);
    Serial.println(F("Fault CLEARED via button. System ARMED."));
    armed = true;
  }
  if (Serial.available()){
    char c=Serial.read();
    if (c=='C'){ faultLatched=false; saveFaultToEEPROM(false); Serial.println(F("Fault CLEARED via Serial. System ARMED.")); armed=true; }
  }

  // Debounced floats
  int top    = topF.read(FLOAT_FILTER_MS);      // LOW = UP (closed)
  int bottom = botF.read(FLOAT_FILTER_MS);      // HIGH = DOWN (open)

  // If fault latched (persisted), keep pump OFF forever (until cleared)
  if (faultLatched) {
    if (pumpRunning) applyPumpState(false);
    setHumid(false); // also disable humidifier on persistent fault
  } else {
    // Normal one-shot logic
    if (!pumpRunning) {
      if (armed && (top == LOW)) {
        applyPumpState(true);
        Serial.println(F("Reason: TOP UP & ARMED -> start pump"));
      }
    } else {
      if (bottom == HIGH) {
        applyPumpState(false);
        armed = false; // disarm after first cycle
        Serial.println(F("Reason: BOTTOM DOWN -> stop pump & DISARM"));
      }
    }

    // Short detect with inrush blanking
    static uint32_t overStart = 0;
    if (pumpRunning) {
      if (millis() - pumpOnAtMs >= FAULT_BLANK_MS) {
        uint32_t mv = readShunt_mV();
        if (mv > SHORT_mV) {
          if (overStart == 0) overStart = millis();
          if (millis() - overStart >= SHORT_MS) {
            Serial.print(F("FAULT: Vshunt=")); Serial.print(mv); Serial.println(F(" mV > 400 mV"));
            applyPumpState(false);
            faultLatched = true;          // latch in RAM
            saveFaultToEEPROM(true);      // persist across resets
            armed = false;
            Serial.println(F("Fault LATCHED and STORED. Pump inhibited until manual CLEAR."));
          }
        } else {
          overStart = 0;
        }
      } else {
        overStart = 0; // ignore inrush
      }
    }

    // HUMID output: ON unless fault (same behavior),
    // but now with reversed electrical polarity for IN2.
    setHumid(!faultLatched);
  }

  // Debug (1 Hz)
  static uint32_t lastDbg=0;
  if (millis() - lastDbg > 1000) {
    lastDbg = millis();
    Serial.print(F("Pump="));  Serial.print(pumpRunning?F("ON"):F("OFF"));
    Serial.print(F("  FaultLatched=")); Serial.print(faultLatched?F("YES"):F("NO"));
    Serial.print(F("  Armed="));Serial.print(armed?F("YES"):F("NO"));
    uint32_t mv = readShunt_mV();
    Serial.print(F("  Vshunt=")); Serial.print(mv); Serial.println(F(" mV"));
  }

  delay(5);
}
