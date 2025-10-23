#include <Arduino.h>
#include <EEPROM.h>

/* ================== USER CONFIG ================== */
// 1 = always clear any stored fault at power-up; 0 = keep it latched across resets
#define AUTO_CLEAR_FAULT_ON_BOOT  1

// Enable a hardware test pin that, when held LOW at power-up, forces a stored fault
#define ENABLE_FAULT_TEST_PIN     1
const int FAULT_TEST_PIN = 5;   // jumper to GND at boot to force fault (only if ENABLE_FAULT_TEST_PIN=1)

/* ======= PINS (Nano/UNO) ======= */
const int FLOAT_TOP_PIN    = 2;   // external pull-up to +5V; switch to GND (LOW when float UP)
const int FLOAT_BOTTOM_PIN = 3;   // external pull-up to +5V; switch to GND (LOW when water present)

const int RELAY_PUMP_PIN   = 8;   // pump relay IN/driver
const int RELAY_HUMID_PIN  = 9;   // humidifier relay IN/driver

// >>> Polarity per relay
const bool RELAY_PUMP_ACTIVE_LOW  = false;  // false=active-HIGH, true=active-LOW
const bool RELAY_HUMID_ACTIVE_LOW = true;   // humidifier module active-LOW

/* ======= FAULT LED ======= */
const int  FAULT_LED_PIN          = 6;     // choose any free digital pin (or LED_BUILTIN)
const bool FAULT_LED_ACTIVE_HIGH  = true;  // HIGH = LED ON (recommended)
inline void setFaultLED(bool on) {
  digitalWrite(
    FAULT_LED_PIN,
    FAULT_LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH)
  );
}

/* ======= ADC (shunt on Node X) ======= */
const int  SHUNT_ADC_PIN    = A0;           // Node X (pump side of 1Ω)
const bool USE_INTERNAL_ADC_REF = true;     // use internal ref for small voltages

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

/* ======= OPTIONAL MANUAL CLEAR BUTTON ======= */
const bool ENABLE_CLEAR_BTN = false;    // set true to use a button
const int  CLEAR_BTN_PIN    = 4;        // external pull-up to +5V; button to GND (pressed = LOW)

/* ======= STATE ======= */
bool pumpRunning  = false;
bool faultLatched = false;  // loaded/managed below
uint32_t lastSwitchMs = 0;
uint32_t pumpOnAtMs   = 0;

/* ================== HELPERS ================== */
inline void setRelayLevel(int pin, bool on, bool activeLow) {
  digitalWrite(pin, (activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW)));
}
inline void relayPumpOn()        { setRelayLevel(RELAY_PUMP_PIN,  true,  RELAY_PUMP_ACTIVE_LOW); }
inline void relayPumpOff()       { setRelayLevel(RELAY_PUMP_PIN,  false, RELAY_PUMP_ACTIVE_LOW); }
inline void setHumid(bool on)    { setRelayLevel(RELAY_HUMID_PIN, on,    RELAY_HUMID_ACTIVE_LOW); }

struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p){
    pin = p;
    pinMode(pin, INPUT);                   // <<< NO INTERNAL PULLUP >>>
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

/* Force/clear/toggle fault from code (used by Serial cmds and boot logic) */
void forceFault(bool on) {
  faultLatched = on;
  saveFaultToEEPROM(on);
  setFaultLED(on);
  if (on) {
    applyPumpState(false); // ensure pump is OFF when fault engages
    Serial.println(F("FORCED: FaultLatched=ON (stored to EEPROM)."));
  } else {
    Serial.println(F("FORCED: FaultLatched=OFF (cleared in EEPROM)."));
  }
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);

  if (ENABLE_FAULT_TEST_PIN) {
    pinMode(FAULT_TEST_PIN, INPUT_PULLUP);          // GND at boot => asserted
  }

  // Inputs with EXTERNAL pull-ups on the board (10k to +5V)
  topF.begin(FLOAT_TOP_PIN);
  botF.begin(FLOAT_BOTTOM_PIN);

  pinMode(RELAY_PUMP_PIN,  OUTPUT);
  pinMode(RELAY_HUMID_PIN, OUTPUT);

  // Fault LED
  pinMode(FAULT_LED_PIN, OUTPUT);
  setFaultLED(false);   // LED OFF at boot

  if (ENABLE_CLEAR_BTN) pinMode(CLEAR_BTN_PIN, INPUT);   // <<< NO PULLUP >>>

  relayPumpOff();
  setHumid(true);     // ON unless fault

  // Optional: force a stored fault at boot via jumper
  if (ENABLE_FAULT_TEST_PIN && digitalRead(FAULT_TEST_PIN) == LOW) {
    saveFaultToEEPROM(true);
    Serial.println(F("BOOT TEST: FAULT_TEST_PIN asserted -> storing FaultLatched=ON."));
  }

  // Power-up fault policy
#if AUTO_CLEAR_FAULT_ON_BOOT
  saveFaultToEEPROM(false);
  faultLatched = false;
  setFaultLED(false);
  Serial.println(F("\nPower-up: clearing any stored fault. FaultLatched=OFF."));
#else
  faultLatched = faultStoredInEEPROM();
  setFaultLED(faultLatched);
  Serial.print(F("\nPower-up: FaultLatched from EEPROM = "));
  Serial.println(faultLatched ? F("ON") : F("OFF"));
#endif

  Serial.print(F("Pump relay trigger: "));  Serial.println(RELAY_PUMP_ACTIVE_LOW  ? F("LOW") : F("HIGH"));
  Serial.print(F("Humid relay trigger: ")); Serial.println(RELAY_HUMID_ACTIVE_LOW ? F("LOW") : F("HIGH"));
  Serial.println(F("Serial cmds: 'F' force fault, 'C' clear, 'T' toggle."));

  // === Startup drain ONLY if bottom float says "water present" ===
  // With your wiring: LOW = float UP (water present).
  delay(FLOAT_FILTER_MS);                         // let inputs settle
  int bottomNow = botF.read(FLOAT_FILTER_MS);

  if (!faultLatched && bottomNow == LOW) {
    Serial.println(F("Startup drain: water detected (bottom float UP). Pump ON until bottom float goes HIGH."));
    // Bypass dwell guard for the first turn-on:
    lastSwitchMs = millis() - MIN_DWELL_MS;
    applyPumpState(true);
  } else {
    Serial.println(F("Startup drain skipped: no water (bottom float DOWN) or fault latched."));
  }
}

/* ================== LOOP ================== */
void loop() {
  // Manual clear button
  if (ENABLE_CLEAR_BTN && checkClearButtonOnce()){
    forceFault(false);
    Serial.println(F("Fault CLEARED via button."));
  }

  // Serial control of fault latch
  if (Serial.available()){
    char c = Serial.read();
    switch (c) {
      case 'C': forceFault(false); break;           // clear fault
      case 'F': forceFault(true);  break;           // set fault
      case 'T': forceFault(!faultLatched); break;   // toggle
      default: break;
    }
  }

  // Debounced floats (LOW = switch closed to GND; HIGH = open via external pull-up)
  int top    = topF.read(FLOAT_FILTER_MS);
  int bottom = botF.read(FLOAT_FILTER_MS);

  if (faultLatched) {
    if (pumpRunning) applyPumpState(false);
    setHumid(false);      // also disable humidifier on persistent fault
    setFaultLED(true);    // LED ON while in fault
  } else {
    // CONTROL — includes startup drain completion
    if (!pumpRunning) {
      if (top == LOW) {                    // TOP UP -> start pump
        applyPumpState(true);
        Serial.println(F("Reason: TOP UP -> start pump"));
      }
    } else {
      if (bottom == HIGH) {                // BOTTOM DOWN (empty) -> stop pump
        applyPumpState(false);
        Serial.println(F("Reason: BOTTOM DOWN -> stop pump"));
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
            saveFaultToEEPROM(true);      // store (policy at boot may clear it)
            setFaultLED(true);            // LED ON
            Serial.println(F("Fault LATCHED and STORED. Pump inhibited until manual CLEAR."));
          }
        } else {
          overStart = 0;
        }
      } else {
        overStart = 0; // ignore inrush
      }
    }

    setHumid(!faultLatched);
    // Keep LED mirrored to current fault state
    setFaultLED(faultLatched);
  }

  // Debug (1 Hz)
  static uint32_t lastDbg=0;
  if (millis() - lastDbg > 1000) {
    lastDbg = millis();
    Serial.print(F("Pump="));  Serial.print(pumpRunning?F("ON"):F("OFF"));
    Serial.print(F("  FaultLatched=")); Serial.print(faultLatched?F("YES"):F("NO"));
    uint32_t mv = readShunt_mV();
    Serial.print(F("  Vshunt=")); Serial.print(mv); Serial.println(F(" mV"));
  }

  delay(5);
}
