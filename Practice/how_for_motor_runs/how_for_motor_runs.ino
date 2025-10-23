#include <Arduino.h>
#include <EEPROM.h>

/* ======= PINS (Nano/UNO) ======= */
const int FLOAT_TOP_PIN    = 2;   // external pull-up to +5V; switch to GND
const int FLOAT_BOTTOM_PIN = 3;   // external pull-up to +5V; switch to GND
const int RELAY_PUMP_PIN   = 8;   // pump relay IN/driver
const int RELAY_HUMID_PIN  = 9;   // humidifier relay IN/driver

// >>> Relay polarity
const bool RELAY_PUMP_ACTIVE_LOW  = false;  // false=active-HIGH, true=active-LOW
const bool RELAY_HUMID_ACTIVE_LOW = true;   // active-LOW module

/* ======= FAULT LED ======= */
const int  FAULT_LED_PIN          = 6;
const bool FAULT_LED_ACTIVE_HIGH  = true;
inline void setFaultLED(bool on) {
  digitalWrite(FAULT_LED_PIN, FAULT_LED_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
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

/* ======= EEPROM ADDRS ======= */
const int EE_ADDR_FAULT     = 0;            // 1 byte
const uint8_t EE_FAULT_CLEAR= 0x00;
const uint8_t EE_FAULT_SET  = 0xF1;
const int EE_ADDR_TOTAL_SEC = 1;            // 4 bytes at 1..4

/* ======= OPTIONAL MANUAL CLEAR BUTTON ======= */
const bool ENABLE_CLEAR_BTN = false;
const int  CLEAR_BTN_PIN    = 4;            // ext pull-up to +5V; button to GND

/* ======= STATE ======= */
bool pumpRunning  = false;
bool faultLatched = false;
uint32_t lastSwitchMs = 0;
uint32_t pumpOnAtMs   = 0;

/* ======= RUNTIME METRICS ======= */
uint32_t lastRunMs   = 0;               // duration of most recent ON cycle (ms)
uint32_t totalRunSec = 0;               // lifetime accumulated ON-time (sec)
inline void saveTotalRunToEEPROM() { static uint32_t lastSaved=0xFFFFFFFF; if (totalRunSec!=lastSaved){ EEPROM.put(EE_ADDR_TOTAL_SEC,totalRunSec); lastSaved=totalRunSec; } }

/* ======= RUN HISTORY (ring buffer) ======= */
struct RunLog {
  uint32_t start_ms;   // millis() when pump turned ON
  uint32_t dur_ms;     // run duration in ms
  char     reason;     // 'B' bottom-empty, 'F' fault, 'U' unknown
};
const uint8_t MAX_RUN_LOGS = 40;
RunLog logs[MAX_RUN_LOGS];
uint16_t logsCount = 0;      // number of valid logs (<= MAX_RUN_LOGS)
uint16_t logsHead  = 0;      // next write index (wraps around)
uint32_t totalOns  = 0;      // how many times pump turned ON
uint32_t totalOffs = 0;      // how many times pump turned OFF

// reason to attribute to the NEXT stop event (set before calling applyPumpState(false))
char pendingStopReason = 'U';

/* ======= HELPERS ======= */
inline void setRelayLevel(int pin, bool on, bool activeLow) {
  digitalWrite(pin, (activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW)));
}
inline void relayPumpOn()        { setRelayLevel(RELAY_PUMP_PIN,  true,  RELAY_PUMP_ACTIVE_LOW); }
inline void relayPumpOff()       { setRelayLevel(RELAY_PUMP_PIN,  false, RELAY_PUMP_ACTIVE_LOW); }
inline void setHumid(bool on)    { setRelayLevel(RELAY_HUMID_PIN, on,    RELAY_HUMID_ACTIVE_LOW); }

struct Debounce {
  int pin, stable, candidate; uint32_t tChange;
  void begin(int p){ pin=p; pinMode(pin, INPUT); stable=candidate=digitalRead(pin); tChange=millis(); }
  int read(uint16_t hold_ms){
    int raw=digitalRead(pin);
    if (raw!=candidate){ candidate=raw; tChange=millis(); }
    if ((millis()-tChange)>=hold_ms && stable!=candidate) stable=candidate;
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

/* ======= LOGGING ======= */
void logRun(uint32_t start_ms, uint32_t dur_ms, char reason) {
  logs[logsHead] = RunLog{start_ms, dur_ms, reason};
  logsHead = (logsHead + 1) % MAX_RUN_LOGS;
  if (logsCount < MAX_RUN_LOGS) logsCount++;
  totalOffs++;
}

/* ======= PUMP STATE APPLY ======= */
void applyPumpState(bool on) {
  if (on == pumpRunning) return;
  if (millis() - lastSwitchMs < MIN_DWELL_MS) return; // dwell guard

  if (on) {
    relayPumpOn();
    pumpOnAtMs = millis();
    totalOns++;
    Serial.println(F("PUMP -> ON (latched)"));
  } else {
    // Measure the just-finished run (only if we were ON)
    if (pumpRunning) {
      uint32_t now = millis();
      lastRunMs = (now >= pumpOnAtMs) ? (now - pumpOnAtMs) : 0;   // wrap-safe
      // lifetime seconds (ceil to avoid undercount)
      uint32_t addSec = (lastRunMs + 999) / 1000;
      totalRunSec += addSec;
      saveTotalRunToEEPROM();

      // Log this run
      logRun(pumpOnAtMs, lastRunMs, pendingStopReason);
      pendingStopReason = 'U'; // reset for next time

      Serial.print(F("PUMP -> OFF (latched). This run: "));
      Serial.print(lastRunMs / 1000.0, 3);
      Serial.print(F(" s  |  Lifetime: "));
      Serial.print(totalRunSec);
      Serial.println(F(" s"));
    }
    relayPumpOff();
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

  // Load lifetime seconds
  EEPROM.get(EE_ADDR_TOTAL_SEC, totalRunSec);
  Serial.print(F("Lifetime pump ON time (EEPROM): "));
  Serial.print(totalRunSec);
  Serial.println(F(" s"));

  topF.begin(FLOAT_TOP_PIN);
  botF.begin(FLOAT_BOTTOM_PIN);

  pinMode(RELAY_PUMP_PIN,  OUTPUT);
  pinMode(RELAY_HUMID_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);
  setFaultLED(false);

  if (ENABLE_CLEAR_BTN) pinMode(CLEAR_BTN_PIN, INPUT);

  relayPumpOff();
  setHumid(true);

  // Clear any stored fault at boot (as requested)
  saveFaultToEEPROM(false);
  faultLatched = false;
  setFaultLED(false);

  Serial.println(F("\nPower-up: clearing any stored fault. FaultLatched=OFF."));
  Serial.print(F("Pump relay trigger: "));  Serial.println(RELAY_PUMP_ACTIVE_LOW  ? F("LOW") : F("HIGH"));
  Serial.print(F("Humid relay trigger: ")); Serial.println(RELAY_HUMID_ACTIVE_LOW ? F("LOW") : F("HIGH"));
  Serial.println(F("Serial cmds: 'C' clear fault, 'R' report, 'H' history, 'X' clear history"));

  // Startup drain ONLY if water present (bottom floaHt LOW)
  delay(FLOAT_FILTER_MS);
  int bottomNow = botF.read(FLOAT_FILTER_MS);
  if (!faultLatched && bottomNow == LOW) {
    Serial.println(F("Startup drain: water detected. Pump ON until bottom float goes HIGH."));
    lastSwitchMs = millis() - MIN_DWELL_MS; // bypass dwell on first turn-on
    applyPumpState(true);
  } else {
    Serial.println(F("Startup drain skipped: no water or fault latched."));
  }
}

/* ======= LOOP ======= */
void loop() {
  // Manual clear button
  if (ENABLE_CLEAR_BTN && checkClearButtonOnce()){
    faultLatched = false; saveFaultToEEPROM(false);
    setFaultLED(false);
    Serial.println(F("Fault CLEARED via button."));
  }

  // Serial commands
  if (Serial.available()){
    char c = Serial.read();
    switch (c) {
      case 'C': // clear fault
        faultLatched=false; saveFaultToEEPROM(false); setFaultLED(false);
        Serial.println(F("Fault CLEARED via Serial."));
        break;
      case 'R': { // runtime report
        Serial.print(F("Pump running: ")); Serial.println(pumpRunning ? F("YES") : F("NO"));
        if (pumpRunning) {
          uint32_t curMs = millis() - pumpOnAtMs;
          Serial.print(F("Current run so far: ")); Serial.print(curMs / 1000.0, 3); Serial.println(F(" s"));
        }
        Serial.print(F("Last completed run: ")); Serial.print(lastRunMs / 1000.0, 3); Serial.println(F(" s"));
        Serial.print(F("Lifetime ON time: ")); Serial.print(totalRunSec); Serial.println(F(" s"));
        Serial.print(F("Total ONs: ")); Serial.print(totalOns); Serial.print(F("  Total OFFs: ")); Serial.println(totalOffs);
      } break;
      case 'H': { // history dump
        Serial.print(F("\n=== RUN HISTORY (latest "));
        Serial.print(logsCount); Serial.println(F(") ==="));
        Serial.println(F("#\tStart[s]\tDur[s]\tReason"));
        // Print oldest -> newest
        uint16_t first = (logsHead + MAX_RUN_LOGS - logsCount) % MAX_RUN_LOGS;
        for (uint16_t i=0; i<logsCount; i++){
          uint16_t idx = (first + i) % MAX_RUN_LOGS;
          Serial.print(i+1); Serial.print('\t');
          Serial.print(logs[idx].start_ms / 1000.0, 3); Serial.print('\t');
          Serial.print(logs[idx].dur_ms   / 1000.0, 3); Serial.print('\t');
          Serial.println(logs[idx].reason);
        }
        Serial.print(F("Totals: ONs=")); Serial.print(totalOns);
        Serial.print(F("  OFFs=")); Serial.println(totalOffs);
        Serial.println(F("=== END HISTORY ===\n"));
      } break;
      case 'X': // clear history
        logsCount=0; logsHead=0; totalOns=0; totalOffs=0;
        Serial.println(F("History cleared."));
        break;
      default: break;
    }
  }

  // Debounced floats (LOW = closed to GND; HIGH = open)
  int top    = topF.read(FLOAT_FILTER_MS);
  int bottom = botF.read(FLOAT_FILTER_MS);

  if (faultLatched) {
    if (pumpRunning) { pendingStopReason='F'; applyPumpState(false); }
    setHumid(false);
    setFaultLED(true);
  } else {
    // Control
    if (!pumpRunning) {
      if (top == LOW) {                  // TOP UP -> start pump
        applyPumpState(true);
        Serial.println(F("Reason: TOP UP -> start pump"));
      }
    } else {
      if (bottom == HIGH) {              // BOTTOM DOWN (empty) -> stop
        pendingStopReason = 'B';
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
            pendingStopReason = 'F';
            applyPumpState(false);
            faultLatched = true;
            saveFaultToEEPROM(true);
            setFaultLED(true);
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
    setFaultLED(faultLatched);
  }

  // Debug (1 Hz)
  static uint32_t lastDbg=0;
  if (millis() - lastDbg > 1000) {
    lastDbg = millis();
    Serial.print(F("Pump="));  Serial.print(pumpRunning?F("ON"):F("OFF"));
    Serial.print(F("  FaultLatched=")); Serial.print(faultLatched?F("YES"):F("NO"));
    uint32_t mv = readShunt_mV();
    Serial.print(F("  Vshunt=")); Serial.print(mv); Serial.print(F(" mV"));
    if (pumpRunning) {
      uint32_t curMs = millis() - pumpOnAtMs;
      Serial.print(F("  RunSoFar=")); Serial.print(curMs / 1000.0, 3); Serial.print(F(" s"));
    }
    Serial.println();
  }

  delay(5);
}
